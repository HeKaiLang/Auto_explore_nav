//==============================================================================
/// @file    image_publisher.cpp
/// @brief   点云→占据栅格地图投影节点 (按需触发)
///
/// 接收 3D 全局点云，通过高度过滤 + 正交投影生成 2D OccupancyGrid。
/// 生成操作由外部服务触发，不自动连续发布。
///
/// @par 订阅话题
///   - <input_topic> (sensor_msgs::PointCloud2) : 输入点云, 默认 /global_map
///
/// @par 发布话题
///   - <output_topic> (nav_msgs::OccupancyGrid, latch) : 输出栅格地图, 默认 /global_map_grid
///
/// @par 提供服务
///   - <trigger_service> (std_srvs::Trigger) : 触发地图生成, 默认 /generate_grid
///
/// @par ROS 参数 (~ 私有命名空间)
///   input_topic    (string,  default: "/global_map")      输入点云话题
///   output_topic   (string,  default: "/global_map_grid") 输出栅格话题
///   trigger_service(string,  default: "/generate_grid")   触发服务名
///   frame_id       (string,  default: "camera_init")      栅格坐标系(输入点云无 frame 时使用)
///   resolution     (double,  default: 0.1)                栅格分辨率 [m]
///   z_min          (double,  default: -1.0)              高度过滤下限 [m]
///   z_max          (double,  default: 1.0)               高度过滤上限 [m]
///
/// @par 用法
///   启动:   rosrun mine_map_system image_publisher
///   触发:   rosservice call /generate_grid "{}"
///   查看:   rostopic echo /global_map_grid
//==============================================================================

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class GridMapPublisher
{
public:
    GridMapPublisher(ros::NodeHandle &nh)
    {
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("input_topic", input_topic_, std::string("/global_map"));
        pnh.param<std::string>("output_topic", output_topic_, std::string("/global_map_grid"));
        pnh.param<std::string>("trigger_service", trigger_service_, std::string("/generate_grid"));
        pnh.param<std::string>("frame_id", frame_id_, std::string("camera_init"));
        pnh.param<double>("resolution", resolution_, 0.1);
        pnh.param<double>("z_min", z_min_, -1.0);
        pnh.param<double>("z_max", z_max_, 1.0);

        sub_ = nh.subscribe(input_topic_, 1, &GridMapPublisher::cloudCallback, this);
        pub_ = nh.advertise<nav_msgs::OccupancyGrid>(output_topic_, 1, true);
        srv_ = nh.advertiseService(trigger_service_, &GridMapPublisher::triggerCallback, this);
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_cloud_ = cloud;
        latest_frame_id_ = msg->header.frame_id;
    }

    bool triggerCallback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZI>());
        std::string cloud_frame;
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (!latest_cloud_ || latest_cloud_->empty())
            {
                res.success = false;
                res.message = "No global map received yet.";
                return true;
            }
            *cloud_copy = *latest_cloud_;
            cloud_frame = latest_frame_id_;
        }

        double min_x = std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();

        std::vector<std::pair<double, double>> points_xy;
        points_xy.reserve(cloud_copy->size());

        for (const auto &pt : cloud_copy->points)
        {
            if (pt.z < z_min_ || pt.z > z_max_)
            {
                continue;
            }
            points_xy.emplace_back(pt.x, pt.y);
            min_x = std::min(min_x, static_cast<double>(pt.x));
            min_y = std::min(min_y, static_cast<double>(pt.y));
            max_x = std::max(max_x, static_cast<double>(pt.x));
            max_y = std::max(max_y, static_cast<double>(pt.y));
        }

        if (points_xy.empty())
        {
            res.success = false;
            res.message = "No points left after height filtering.";
            return true;
        }

        if (resolution_ <= 0.0)
        {
            res.success = false;
            res.message = "Resolution must be positive.";
            return true;
        }

        const int width = static_cast<int>(std::floor((max_x - min_x) / resolution_)) + 1;
        const int height = static_cast<int>(std::floor((max_y - min_y) / resolution_)) + 1;
        if (width <= 0 || height <= 0)
        {
            res.success = false;
            res.message = "Invalid grid size computed.";
            return true;
        }

        nav_msgs::OccupancyGrid grid;
        grid.header.stamp = ros::Time::now();
        grid.header.frame_id = cloud_frame.empty() ? frame_id_ : cloud_frame;
        grid.info.resolution = resolution_;
        grid.info.width = static_cast<uint32_t>(width);
        grid.info.height = static_cast<uint32_t>(height);
        grid.info.origin.position.x = min_x;
        grid.info.origin.position.y = min_y;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(static_cast<size_t>(width * height), -1);

        for (const auto &xy : points_xy)
        {
            int ix = static_cast<int>(std::floor((xy.first - min_x) / resolution_));
            int iy = static_cast<int>(std::floor((xy.second - min_y) / resolution_));
            if (ix < 0 || iy < 0 || ix >= width || iy >= height)
            {
                continue;
            }
            const size_t index = static_cast<size_t>(iy * width + ix);
            grid.data[index] = 100;
        }

        pub_.publish(grid);
        res.success = true;
        res.message = "Occupancy grid published.";
        return true;
    }

    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::ServiceServer srv_;

    std::mutex cloud_mutex_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud_;
    std::string latest_frame_id_;

    std::string input_topic_;
    std::string output_topic_;
    std::string trigger_service_;
    std::string frame_id_;
    double resolution_ = 0.1;
    double z_min_ = -1.0;
    double z_max_ = 1.0;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    GridMapPublisher node(nh);
    ROS_INFO("image publisher started");

    ros::spin();
    return 0;
}
