#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <algorithm>

/**
 * @brief cmd_vel → 2字节控制指令 转换节点
 * 
 * 订阅 /cmd_vel
 * 发布 /robot_control_bytes (std_msgs/UInt8MultiArray, 2 bytes)
 * 
 * 控制策略：
 *   1. 由 v 和 ω 计算机体曲率 κ = ω / v
 *   2. 由曲率 κ 反解关节偏转角 σ（论文3.3节公式）
 *   3. 由 σ 计算所需绳长差，进而得到舵机A转角
 *   4. 线速度 v 直接映射为舵机B转速PWM
 */
class CmdVelToControl
{
public:
    CmdVelToControl(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        /* ---------- 加载参数 ---------- */
        pnh.param("robot_geometry/n_spine_segments", n_spine_, 10);
        pnh.param("robot_geometry/spine_pitch_P", P_, 0.02275);
        pnh.param("robot_geometry/spine_d1", d1_, 0.0345);
        pnh.param("robot_geometry/spine_d2", d2_, 0.00455);
        pnh.param("robot_geometry/spine_alpha", alpha_, 0.0872665);

        pnh.param("steering_servo/drum_radius", drum_radius_, 0.005);
        pnh.param("steering_servo/steering_servo_ratio", steering_ratio_, 1.0);
        pnh.param("steering_servo/pwm_mid", pwm_mid_steering_, 128);
        pnh.param("steering_servo/pwm_to_servo_angle", pwm_to_servo_angle_, 0.0122718);
        pnh.param("steering_servo/max_deflection", max_sigma_, 0.174533);

        pnh.param("drive_servo/pwm_mid", pwm_mid_drive_, 128);
        pnh.param("drive_servo/pwm_to_velocity", pwm_to_vel_, 0.00625);

        pnh.param("control_limits/max_linear_vel", max_v_, 0.8);
        pnh.param("control_limits/max_angular_vel", max_omega_, 2.0);
        pnh.param("control_limits/min_linear_vel_for_steering", min_v_threshold_, 0.05);

        cmd_vel_sub_ = nh.subscribe("/cmd_vel", 10, &CmdVelToControl::cmdVelCallback, this);
        control_pub_ = nh.advertise<std_msgs::UInt8MultiArray>("/robot_control_bytes", 10);

        ROS_INFO("cmdvel_to_control_node initialized.");
    }

    /**
     * @brief 由曲率 κ 反求关节偏转角 σ
     * 
     * 论文3.3节：κ = 2*tan(σ/2) / (2*P + d2)
     * 反解：σ = 2 * atan( κ * (2*P + d2) / 2 )
     */
    void curvatureToJointDeflection(double curvature, double& joint_angle)
    {
        if (std::abs(curvature) < 1e-6) {
            joint_angle = 0.0;
        } else {
            joint_angle = 2.0 * std::atan(curvature * (2.0 * P_ + d2_) / 2.0);
            // 饱和限幅
            if (joint_angle > max_sigma_) joint_angle = max_sigma_;
            if (joint_angle < -max_sigma_) joint_angle = -max_sigma_;
        }
    }

    /**
     * @brief 由关节偏转角 σ 求舵机A转动角度
     * 
     * 论文3.3节绳长关系：
     *   总绳长差 = 2*(2+n)*d1*sin(σ/2) = 2 * R_drum * θ_servo
     * 因此：θ_servo = (2+n)*d1*sin(σ/2) / R_drum
     */
    void jointDeflectionToServoAngle(double joint_angle, double& servo_angle)
    {
        double half_sigma = joint_angle / 2.0;
        double servo_angle_out = (2.0 + static_cast<double>(n_spine_)) * d1_ * std::sin(half_sigma) / drum_radius_;
        // 换算到电机端（考虑减速比）
        servo_angle = servo_angle_out * steering_ratio_;
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        double v = msg->linear.x;
        double omega = msg->angular.z;

        /* ---- 速度限幅 ---- */
        v = std::max(-max_v_, std::min(max_v_, v));
        omega = std::max(-max_omega_, std::min(max_omega_, omega));

        /* ---- 计算曲率 κ = ω / v ---- */
        double kappa = 0.0;
        if (std::abs(v) > min_v_threshold_) {
            kappa = omega / v;
        } else {
            // 低速或零速时：若存在角速度指令，赋予一个最小线速度符号以保留转向方向
            if (std::abs(omega) > 0.01) {
                double v_sign = (v >= 0.0) ? 1.0 : -1.0;
                v = min_v_threshold_ * v_sign;
                kappa = omega / v;
            }
        }

        // 最大曲率限制（对应论文最小转弯半径 r_min）
        double max_kappa = 2.0 * std::tan(max_sigma_ / 2.0) / (2.0 * P_ + d2_);
        kappa = std::max(-max_kappa, std::min(max_kappa, kappa));

        /* ---- 反解转向舵机A ---- */
        double sigma;
        curvatureToJointDeflection(kappa, sigma);

        double servo_angle_A;
        jointDeflectionToServoAngle(sigma, servo_angle_A);

        // 舵机转角 → PWM
        int pwm_A = pwm_mid_steering_ + static_cast<int>(std::round(servo_angle_A / pwm_to_servo_angle_));

        /* ---- 映射驱动舵机B（线速度） ---- */
        int pwm_B = pwm_mid_drive_ + static_cast<int>(std::round(v / pwm_to_vel_));

        /* ---- 限幅到 0~255 ---- */
        pwm_A = std::max(0, std::min(255, pwm_A));
        pwm_B = std::max(0, std::min(255, pwm_B));

        /* ---- 发布 2字节控制指令 ---- */
        std_msgs::UInt8MultiArray control_msg;
        control_msg.data.resize(2);
        control_msg.data[0] = static_cast<uint8_t>(pwm_A);
        control_msg.data[1] = static_cast<uint8_t>(pwm_B);
        control_pub_.publish(control_msg);

        ROS_DEBUG("cmd_vel[v=%.3f, w=%.3f] → kappa=%.3f, sigma=%.3f rad, servoA=%.3f rad, PWM[%d, %d]",
                  msg->linear.x, msg->angular.z, kappa, sigma, servo_angle_A, pwm_A, pwm_B);
    }

private:
    /* 几何参数 */
    int n_spine_;
    double P_, d1_, d2_, alpha_;

    /* 转向舵机A参数 */
    double drum_radius_;
    double steering_ratio_;
    int pwm_mid_steering_;
    double pwm_to_servo_angle_;
    double max_sigma_;

    /* 驱动舵机B参数 */
    int pwm_mid_drive_;
    double pwm_to_vel_;

    /* 控制限幅 */
    double max_v_, max_omega_, min_v_threshold_;

    /* ROS 成员 */
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher control_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmdvel_to_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    CmdVelToControl node(nh, pnh);
    ros::spin();
    return 0;
}