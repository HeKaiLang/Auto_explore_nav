// control_to_cmdvel_node.cpp
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>

class ControlToCmdVel
{
public:
    ControlToCmdVel(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        pnh.param("scale_linear", scale_linear_, 0.05);      // 每字节偏移对应 m/s
        pnh.param("scale_angular", scale_angular_, 0.25);     // 每字节偏移对应 rad/s
        pnh.param("byte_mid", byte_mid_, 0x0a);               // 中位字节

        sub_ = nh.subscribe("/robot_control_bytes", 10, &ControlToCmdVel::callback, this);
        pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_joy", 10);
    }

    void callback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
    {
        if (msg->data.size() < 5) return;

        uint8_t b_qianjin = msg->data[3];   // 切向速度字节
        uint8_t b_zuozhuan = msg->data[4]; // 转向速度字节

        geometry_msgs::Twist twist;
        twist.linear.x  = scale_linear_  * (static_cast<double>(b_qianjin)   - byte_mid_);
        twist.linear.y  = 0.0;
        twist.linear.z  = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = scale_angular_ * (static_cast<double>(b_zuozhuan) - byte_mid_);

        pub_.publish(twist);
    }

private:
    double scale_linear_, scale_angular_;
    int byte_mid_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_to_cmdvel_node");
    ros::NodeHandle nh, pnh("~");
    ControlToCmdVel node(nh, pnh);
    ros::spin();
    return 0;
}