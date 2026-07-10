// cmdvel_to_serial_node.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

class CmdVelToSerial
{
public:
    CmdVelToSerial(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        // 加载参数
        pnh.param("serial_port", port_, std::string("/dev/ttyUSB0"));
        pnh.param("serial_baudrate", baudrate_, 115200);
        pnh.param("byte_mid", byte_mid_, 0x0a);
        pnh.param("scale_linear_inv", scale_linear_inv_, 20.0);   // 1 / scale_linear
        pnh.param("scale_angular_inv", scale_angular_inv_, 4.0);  // 1 / scale_angular

        // PID 参数（角速度闭环）
        pnh.param("pid_kp", kp_, 2.0);
        pnh.param("pid_ki", ki_, 0.1);
        pnh.param("pid_kd", kd_, 0.0);
        pnh.param("pid_integral_max", integral_max_, 5.0);

        // 绳长保护参数
        pnh.param("rope_max_mm", rope_max_mm_, 36.0);
        pnh.param("rope_k_scale", rope_k_scale_, 0.05);  // mm/字节/loop
        pnh.param("loop_rate_hz", loop_rate_hz_, 100.0);

        // 初始化串口
        ser_.setPort(port_);
        ser_.setBaudrate(baudrate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(20);
        ser_.setTimeout(to);
        try {
            ser_.open();
            ROS_INFO("Serial opened: %s @ %d", port_.c_str(), baudrate_);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to open serial: %s", e.what());
            ros::shutdown();
        }

        // 订阅
        cmd_vel_sub_ = nh.subscribe("/cmd_vel", 10, &CmdVelToSerial::cmdVelCb, this);
        imu_sub_ = nh.subscribe("/imu/data", 10, &CmdVelToSerial::imuCb, this);  // 改成你的 IMU 话题

        // 初始化停车指令
        stop_cmd_[0] = 0xff;
        stop_cmd_[1] = byte_mid_;
        stop_cmd_[2] = byte_mid_;
        stop_cmd_[3] = byte_mid_;
        stop_cmd_[4] = byte_mid_;

        // 上电回中：先发送 1 秒中位，让绳子松弛
        ROS_INFO("Homing steering servo for 1.0s...");
        ros::Rate r(100);
        for (int i = 0; i < 100 && ros::ok(); ++i) {
            ser_.write(stop_cmd_, 5);
            r.sleep();
        }
        ROS_INFO("Homing done. Starting control loop.");
    }

    void cmdVelCb(const geometry_msgs::Twist::ConstPtr& msg)
    {
        cmd_vel_ = *msg;
        has_cmd_vel_ = true;
    }

    void imuCb(const sensor_msgs::Imu::ConstPtr& msg)
    {
        imu_yaw_rate_ = msg->angular_velocity.z;
        has_imu_ = true;
    }

    // 主循环，在 main 中调用
    void run()
    {
        ros::Rate rate(loop_rate_hz_);
        while (ros::ok())
        {
            ros::spinOnce();

            uint8_t out[5];
            out[0] = 0xff;
            out[1] = byte_mid_;  // taitou 保持中位（暂不参与）
            out[2] = byte_mid_;  // shifang 保持中位

            // ---- 1. 线速度开环映射 ----
            double v = cmd_vel_.linear.x;
            int byte_v = byte_mid_ + static_cast<int>(std::round(v * scale_linear_inv_));
            byte_v = std::max(0, std::min(255, byte_v));
            out[3] = static_cast<uint8_t>(byte_v);

            // ---- 2. 角速度：IMU PID 闭环 ----
            int byte_w_raw;
            if (has_cmd_vel_) {
                double e = cmd_vel_.angular.z - imu_yaw_rate_;  // 期望 - 反馈
                integral_ += e / loop_rate_hz_;
                integral_ = std::max(-integral_max_, std::min(integral_max_, integral_));

                double derivative = 0.0;
                if (has_imu_) {
                    derivative = -(imu_yaw_rate_ - last_yaw_rate_) * loop_rate_hz_; // d(error)/dt ≈ -d(feedback)/dt
                }
                last_yaw_rate_ = imu_yaw_rate_;

                double u = kp_ * e + ki_ * integral_ + kd_ * derivative;

                // 叠加到开环前馈上（可选：纯闭环也行，这里用前馈+反馈）
                double w_feedforward = cmd_vel_.angular.z * scale_angular_inv_;
                byte_w_raw = byte_mid_ + static_cast<int>(std::round(w_feedforward + u));
            } else {
                byte_w_raw = byte_mid_;
            }

            // ---- 3. 绳长保护（软件限幅）----
            double dt = 1.0 / loop_rate_hz_;
            rope_delta_ += rope_k_scale_ * (static_cast<double>(byte_w_raw) - byte_mid_) * dt;
            rope_delta_ = std::max(-rope_max_mm_, std::min(rope_max_mm_, rope_delta_));

            // 如果达到限幅，强制停止同向转向
            bool saturated = false;
            if (rope_delta_ >= rope_max_mm_ && (byte_w_raw - byte_mid_) > 0) {
                byte_w_raw = byte_mid_;
                saturated = true;
            } else if (rope_delta_ <= -rope_max_mm_ && (byte_w_raw - byte_mid_) < 0) {
                byte_w_raw = byte_mid_;
                saturated = true;
            }

            int byte_w = std::max(0, std::min(255, byte_w_raw));
            out[4] = static_cast<uint8_t>(byte_w);

            // 发布调试（可选）
            if (debug_pub_.getNumSubscribers() > 0) {
                std_msgs::UInt8MultiArray dbg;
                dbg.data.assign(out, out + 5);
                debug_pub_.publish(dbg);
            }

            // 写串口
            if (ser_.isOpen()) {
                ser_.write(out, 5);
            }

            // 如果 cmd_vel 长期不来（>0.2s），发停车
            if ((ros::Time::now() - last_cmd_vel_time_).toSec() > 0.2) {
                ser_.write(stop_cmd_, 5);
            }
            if (has_cmd_vel_) last_cmd_vel_time_ = ros::Time::now();

            rate.sleep();
        }

        // 退出前停车
        if (ser_.isOpen()) ser_.write(stop_cmd_, 5);
    }

private:
    // 参数
    std::string port_;
    int baudrate_, byte_mid_;
    double scale_linear_inv_, scale_angular_inv_;
    double kp_, ki_, kd_, integral_max_;
    double rope_max_mm_, rope_k_scale_, loop_rate_hz_;

    // 状态
    geometry_msgs::Twist cmd_vel_;
    bool has_cmd_vel_ = false;
    bool has_imu_ = false;
    double imu_yaw_rate_ = 0.0;
    double last_yaw_rate_ = 0.0;
    double integral_ = 0.0;
    double rope_delta_ = 0.0;
    ros::Time last_cmd_vel_time_;

    // 串口
    serial::Serial ser_;
    uint8_t stop_cmd_[5];

    // ROS
    ros::Subscriber cmd_vel_sub_, imu_sub_;
    ros::Publisher debug_pub_;  // 如需调试可发布出去
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmdvel_to_serial_node");
    ros::NodeHandle nh, pnh("~");
    CmdVelToSerial node(nh, pnh);
    node.run();
    return 0;
}