//==============================================================================
/// @file    selected_marker_nav.cpp
/// @brief   订阅 /selected_markers，取第一个标记点作为导航目标
///
/// 订阅 /selected_markers (visualization_msgs::Marker)，提取第一个点的坐标，
/// 通过 move_base_flex 的 action 接口发送导航目标，使机器人移动到该点。
///
/// @par 订阅话题
///   - /selected_markers (visualization_msgs::Marker) : 手柄端选中的标记点
///
/// @par Action 客户端
///   - /move_base_flex/move_base (move_base_flex_msgs::MoveBaseAction) : 导航目标
///
/// @par 行为
///   - 仅取第一个标记点作为导航目标
///   - 直接使用 camera_init 坐标，不做 TF 转换
///   - 收到新标记时，如果当前有导航任务会先取消再发送新目标
///   - 发送导航目标后，等待到达或失败（不阻塞订阅）
///
/// @par 用法
///   启动: rosrun mine_map_deploy selected_marker_nav
//==============================================================================

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_flex_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

class SelectedMarkerNav
{
public:
    SelectedMarkerNav()
        : ac_("/move_base_flex/move_base", true)
        , has_active_goal_(false)
        , active_goal_index_(0)
    {
        ros::NodeHandle nh;

        // 等待 action server 启动
        ROS_INFO("Waiting for move_base_flex action server...");
        if (!ac_.waitForServer(ros::Duration(10.0)))
        {
            ROS_WARN("move_base_flex action server not available after 10s, will keep trying...");
        }
        else
        {
            ROS_INFO("Connected to move_base_flex action server.");
        }

        // 订阅 /selected_markers
        sub_ = nh.subscribe<visualization_msgs::Marker>(
            "/selected_markers", 1, &SelectedMarkerNav::markerCallback, this);

        ROS_INFO("selected_marker_nav ready, listening on /selected_markers");
    }

private:
    void markerCallback(const visualization_msgs::Marker::ConstPtr& msg)
    {
        // 检查是否有有效点
        if (msg->points.empty())
        {
            ROS_WARN("Received empty marker, ignoring.");
            return;
        }

        const auto& first_pt = msg->points[0];
        std::string frame_id = msg->header.frame_id;
        if (frame_id.empty())
        {
            frame_id = "camera_init";
        }

        ROS_INFO("Received marker with %zu points, navigating to first: (%.2f, %.2f, %.2f) [%s]",
                 msg->points.size(), first_pt.x, first_pt.y, first_pt.z, frame_id.c_str());

        // 构造导航目标
        move_base_flex_msgs::MoveBaseGoal goal;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = frame_id;
        goal.target_pose.pose.position.x = first_pt.x;
        goal.target_pose.pose.position.y = first_pt.y;
        goal.target_pose.pose.position.z = first_pt.z;

        // 朝向保持默认 (四元数 identity, 即不关心朝向)
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;

        // 如果之前有活跃目标，先取消
        if (has_active_goal_)
        {
            ROS_INFO("Cancelling previous navigation goal...");
            ac_.cancelGoal();
            has_active_goal_ = false;
        }

        // 递增目标序号 (仅用于日志追踪)
        active_goal_index_++;

        // 发送导航目标
        ac_.sendGoal(
            goal,
            boost::bind(&SelectedMarkerNav::doneCallback, this, _1, _2, active_goal_index_),
            boost::bind(&SelectedMarkerNav::activeCallback, this, active_goal_index_),
            boost::bind(&SelectedMarkerNav::feedbackCallback, this, _1, active_goal_index_));

        has_active_goal_ = true;
        ROS_INFO("Navigation goal #%d sent to move_base_flex.", active_goal_index_);
    }

    void doneCallback(const actionlib::SimpleClientGoalState& state,
                      const move_base_flex_msgs::MoveBaseResultConstPtr& result,
                      int goal_index)
    {
        has_active_goal_ = false;

        std::string outcome_text = "UNKNOWN";
        if (result)
        {
            outcome_text = result->outcome == 0 ? "SUCCEEDED" 
                         : result->outcome == 1 ? "FAILURE" 
                         : result->outcome == 2 ? "CANCELED" 
                         : "UNKNOWN";
        }

        ROS_INFO("Navigation goal #%d finished: state=%s, outcome=%s",
                 goal_index, state.toString().c_str(), outcome_text.c_str());
    }

    void activeCallback(int goal_index)
    {
        ROS_INFO("Navigation goal #%d is now active (robot moving).", goal_index);
    }

    void feedbackCallback(const move_base_flex_msgs::MoveBaseFeedbackConstPtr& feedback,
                          int goal_index)
    {
        ROS_DEBUG("Navigation goal #%d feedback: state=%d",
                  goal_index, feedback ? feedback->state : -1);
    }

    ros::Subscriber sub_;
    actionlib::SimpleActionClient<move_base_flex_msgs::MoveBaseAction> ac_;
    bool has_active_goal_;
    int active_goal_index_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "selected_marker_nav");
    SelectedMarkerNav node;
    ros::spin();
    return 0;
}
