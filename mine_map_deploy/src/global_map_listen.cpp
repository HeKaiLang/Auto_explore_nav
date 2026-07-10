//==============================================================================
/// @file    global_map_listen.cpp
/// @brief   全局点云累积与发布节点 (体素哈希表)
///
/// 订阅 /cloud_registered 接收 SLAM 前端输出的局部点云，增量累积到全局体素哈希表。
/// 体素哈希表天然去重，内存仅与已探索体积相关，不随时间无限增长。
/// 定时发布全局点云、按需导出 PCD 文件。
///
/// @par 订阅话题
///   - /cloud_registered (sensor_msgs::PointCloud2) : SLAM 前端输出的配准点云
///
/// @par 发布话题 (可选, 受 publish_enable 控制)
///   - <publish_topic> (sensor_msgs::PointCloud2, latch) : 全局累积点云, 默认 /global_map
///
/// @par 提供服务
///   - /save_global_map (std_srvs::Trigger) : 将当前全局地图保存为 PCD 文件
///
/// @par ROS 参数 (~ 私有命名空间)
///   leaf_size        (float,   default: 0.1)          体素降采样栅格尺寸 [m]
///   save_dir         (string,  default: ".")           PCD 文件保存目录
///   publish_enable   (bool,    default: true)          是否启用定时发布
///   publish_topic    (string,  default: "/global_map") 发布话题名
///   publish_rate     (double,  default: 1.0)           发布频率 [Hz]
///   max_voxel_count  (size_t,  default: 0)             体素数量上限 (0=不限)
///
/// @par 用法
///   启动:   rosrun mine_map_system global_map_listen
///   保存:   rosservice call /save_global_map "{}"
///   查看:   rostopic echo /global_map
///
/// @note  替代了旧版无界累积 + 全量 VoxelGrid 滤波方案
//==============================================================================

#include <cmath>
#include <mutex>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// ── 体素键：用于哈希表去重 ──────────────────────────────────
struct VoxelKey
{
	int ix, iy, iz;
	bool operator==(const VoxelKey &o) const
	{
		return ix == o.ix && iy == o.iy && iz == o.iz;
	}
};

struct VoxelKeyHash
{
	size_t operator()(const VoxelKey &k) const
	{
		return (static_cast<size_t>(k.ix) * 73856093) ^
		       (static_cast<size_t>(k.iy) * 19349663) ^
		       (static_cast<size_t>(k.iz) * 83492791);
	}
};

class GlobalMapListener
{
public:
	GlobalMapListener(ros::NodeHandle &nh)
	{
		ros::NodeHandle pnh("~");
		pnh.param<float>("leaf_size", leaf_size_, 0.1f);
		if (leaf_size_ <= 0.0f)
		{
			ROS_WARN("leaf_size=%.3f invalid, fallback to 0.1", leaf_size_);
			leaf_size_ = 0.1f;
		}
		pnh.param<std::string>("save_dir", save_dir_, std::string("."));
		pnh.param<bool>("publish_enable", publish_enable_, true);
		pnh.param<std::string>("publish_topic", publish_topic_, std::string("/global_map"));
		pnh.param<double>("publish_rate", publish_rate_, 1.0);
		// 安全上限：达到后拒绝新体素，防止极端场景 OOM（0 = 不限）
		int max_voxel_count_tmp = 0;
		pnh.param<int>("max_voxel_count", max_voxel_count_tmp, 0);
		max_voxel_count_ = static_cast<size_t>(max_voxel_count_tmp);

		voxel_map_.reserve(1 << 20);  // 预分配 1M 桶，减少 rehash
		latest_publish_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

		sub_ = nh.subscribe("/cloud_registered", 100, &GlobalMapListener::cloudCallback, this);
		srv_ = nh.advertiseService("/save_global_map", &GlobalMapListener::saveService, this);
		if (publish_enable_ && publish_rate_ > 0.0)
		{
			pub_ = nh.advertise<sensor_msgs::PointCloud2>(publish_topic_, 1, true);
			publish_timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_),
							   &GlobalMapListener::publishTimerCallback, this);
		}
	}

private:
	// ── 点云回调：增量插入体素哈希表 ───────────────────────
	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::fromROSMsg(*msg, *cloud);

		const float inv_leaf = 1.0f / leaf_size_;
		size_t added = 0;

		std::lock_guard<std::mutex> lock(map_mutex_);
		for (const auto &pt : cloud->points)
		{
			// 达到上限时停止插入，但仍可更新已有体素
			if (max_voxel_count_ > 0 && voxel_map_.size() >= max_voxel_count_)
			{
				auto it = voxel_map_.find(voxelKey(pt, inv_leaf));
				if (it != voxel_map_.end())
				{
					it->second = pt;  // 用最新点更新
				}
				continue;
			}

			auto result = voxel_map_.insert({voxelKey(pt, inv_leaf), pt});
			if (result.second)
			{
				++added;
			}
			else
			{
				// 同一体素已存在，用最新点覆盖（保留最新观测）
				result.first->second = pt;
			}
		}
		++frame_count_;

		// 每 500 帧打印一次统计
		if (frame_count_ % 500 == 0)
		{
			ROS_INFO("[global_map] frames=%d  voxels=%zu  added_this_frame=%zu",
			         frame_count_, voxel_map_.size(), added);
		}
	}

	// ── 保存服务：从哈希表导出 PCD ───────────────────────────
	bool saveService(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = hashToCloud();

		if (cloud->empty())
		{
			res.success = false;
			res.message = "Global map is empty.";
			return true;
		}

		const uint64_t stamp_ns = ros::Time::now().toNSec();
		const std::string file_path = save_dir_ + "/global_map_" + std::to_string(stamp_ns) + ".pcd";

		pcl::PCDWriter writer;
		int ret = writer.writeBinaryCompressed(file_path, *cloud);
		if (ret != 0)
		{
			res.success = false;
			res.message = "Failed to save map to " + file_path;
			return true;
		}

		res.success = true;
		res.message = "Saved " + std::to_string(cloud->size()) + " points to " + file_path;
		return true;
	}

	// ── 定时发布：仅当哈希表有变化时才重新生成消息 ──────────
	void publishTimerCallback(const ros::TimerEvent &)
	{
		{
			std::lock_guard<std::mutex> lock(map_mutex_);
			if (voxel_map_.empty() || voxel_map_.size() == published_voxel_count_)
			{
				return;  // 无新数据，跳过
			}
			published_voxel_count_ = voxel_map_.size();
		}

		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = hashToCloud();
		if (cloud->empty()) return;

		sensor_msgs::PointCloud2 msg;
		pcl::toROSMsg(*cloud, msg);
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "camera_init";
		pub_.publish(msg);

		// 缓存最新发布的云，供外部零拷贝读取（可选）
		std::lock_guard<std::mutex> lock(pub_mutex_);
		latest_publish_cloud_.swap(cloud);
	}

	// ── 工具：体素坐标计算 ──────────────────────────────────
	static inline VoxelKey voxelKey(const pcl::PointXYZI &pt, float inv_leaf)
	{
		return {
			static_cast<int>(std::floor(pt.x * inv_leaf)),
			static_cast<int>(std::floor(pt.y * inv_leaf)),
			static_cast<int>(std::floor(pt.z * inv_leaf))
		};
	}

	// ── 工具：哈希表 → PointCloud ───────────────────────────
	pcl::PointCloud<pcl::PointXYZI>::Ptr hashToCloud()
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
		std::lock_guard<std::mutex> lock(map_mutex_);
		cloud->reserve(voxel_map_.size());
		for (const auto &kv : voxel_map_)
		{
			cloud->push_back(kv.second);
		}
		return cloud;
	}

	// ── 成员变量 ────────────────────────────────────────────
	ros::Subscriber sub_;
	ros::ServiceServer srv_;
	ros::Publisher pub_;
	ros::Timer publish_timer_;

	std::unordered_map<VoxelKey, pcl::PointXYZI, VoxelKeyHash> voxel_map_;
	std::mutex map_mutex_;

	pcl::PointCloud<pcl::PointXYZI>::Ptr latest_publish_cloud_;
	std::mutex pub_mutex_;

	int frame_count_ = 0;
	size_t published_voxel_count_ = 0;
	size_t max_voxel_count_ = 0;
	float leaf_size_ = 0.1f;
	std::string save_dir_;
	bool publish_enable_ = true;
	std::string publish_topic_;
	double publish_rate_ = 1.0;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "global_map_listener");
	ros::NodeHandle nh;

	GlobalMapListener node(nh);
	ros::spin();
	return 0;
}
