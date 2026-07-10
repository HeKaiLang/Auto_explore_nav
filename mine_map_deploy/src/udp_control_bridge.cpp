// udp_control_bridge.cpp
// 原：my_serial_port_control
// 现：只负责接收 UDP 手柄数据，发布 /robot_control_bytes

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>   // 【新增】
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <cmath>
#include <fcntl.h>

using namespace std;

#pragma pack(push, 1)
struct INFO
{
    uint8_t header;
    uint8_t taitou_12;
    uint8_t shifang_4;
    uint8_t qianjin_16;
    uint8_t zuozhuan_17;
};
#pragma pack(pop)

INFO dc;
uint8_t recv_info_buf[sizeof(INFO)];
uint8_t stop_cmd[5] = {0xff, 0x0a, 0x0a, 0x0a, 0x0a};
uint8_t data[5] = {0xff, 0x0a, 0x0a, 0x0a, 0x0a};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_control_bridge");
    ros::NodeHandle n;

    // 【新增】发布 /robot_control_bytes
    ros::Publisher control_pub = n.advertise<std_msgs::UInt8MultiArray>(
        "/robot_control_bytes", 10);

    // ========== 串口代码保留但不再使用，方便回退 ==========
    serial::Serial ser;
    ser.setPort("/dev/ttyUSB0");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(200);
    ser.setTimeout(to);
    try {
        ser.open();
    } catch (const std::exception& e) {
        ROS_WARN("Serial port /dev/ttyUSB0 not available, running in bridge-only mode.");
    }
    // =====================================================

    // UDP 初始化（保持原样）
    struct sockaddr_in Server;
    int serverSocket = socket(PF_INET, SOCK_DGRAM, 0);
    int flags = fcntl(serverSocket, F_GETFL, 0);
    fcntl(serverSocket, F_SETFL, flags | O_NONBLOCK);

    Server.sin_family = AF_INET;
    Server.sin_addr.s_addr = inet_addr("192.168.166.49");
    Server.sin_port = htons(7950);

    if (bind(serverSocket, (sockaddr *)&Server, sizeof(sockaddr)) < 0) {
        ROS_ERROR("Socket Bind Failed!");
        return -1;
    }

    ros::Rate loop_rate(100);
    ros::Time last_msg_time = ros::Time::now();
    ROS_INFO("udp_control_bridge running. Publishing to /robot_control_bytes");

    while (ros::ok())
    {
        struct sockaddr_in sender_addr;
        socklen_t len = sizeof(sender_addr);
        int tt = recvfrom(serverSocket, recv_info_buf, sizeof(recv_info_buf), 0,
                          (sockaddr *)&sender_addr, &len);

        if (tt > 0)
        {
            last_msg_time = ros::Time::now();
            memcpy(&dc, recv_info_buf, sizeof(recv_info_buf));

            int aa = static_cast<int>(dc.header);
            int bb = static_cast<int>(dc.taitou_12);
            int cc = static_cast<int>(dc.shifang_4);
            int dd = static_cast<int>(dc.qianjin_16);
            int ee = static_cast<int>(dc.zuozhuan_17);

            if (aa == 255)
            {
                data[0] = aa & 0xff;

                // data[1] taitou
                if (bb < 110) {
                    bb = 10 - floor(bb / 11.0);
                    data[1] = bb & 0xff;
                } else if (bb > 130) {
                    bb = 10 + floor((bb - 130) / 12.5);
                    data[1] = bb & 0xff;
                } else {
                    data[1] = 0x0a;
                }

                // data[2] shifang
                if (cc < 50) {
                    data[2] = 0x00;
                } else if (cc > 200) {
                    data[2] = 0xff;
                } else {
                    data[2] = 0x0a;
                }

                // data[3] qianjin
                if (dd < 110) {
                    dd = floor(1.25 * dd - 27.5);
                    dd = floor(dd / 11.0);
                    data[3] = dd & 0xff;
                } else if (dd > 130) {
                    dd = floor(1.53 * dd - 63.6);
                    dd = 10 + floor((dd - 120) / 12.5);
                    data[3] = dd & 0xff;
                } else {
                    data[3] = 0x0a;
                }

                // data[4] zuozhuan
                if (ee < 110) {
                    ee = floor(1.25 * ee - 27.5);
                    ee = floor(ee / 11.0);
                    data[4] = ee & 0xff;
                } else if (ee > 130) {
                    ee = floor(1.53 * ee - 63.6);
                    ee = 10 + floor((ee - 120) / 12.5);
                    data[4] = ee & 0xff;
                } else {
                    data[4] = 0x0a;
                }

                // 【新增】发布到 ROS 话题
                std_msgs::UInt8MultiArray msg;
                msg.data.assign(data, data + 5);
                control_pub.publish(msg);

                // 【注释掉】不再直接写串口，由 cmdvel_to_serial_node 处理
                // ser.write(data, 5);
            }
            else
            {
                // Header 不对，发布停车指令（供下游节点判断超时）
                std_msgs::UInt8MultiArray msg;
                msg.data.assign(stop_cmd, stop_cmd + 5);
                control_pub.publish(msg);
            }
        }
        else
        {
            // 超时保护：超过 0.2s 没收到数据，发布停车
            if ((ros::Time::now() - last_msg_time).toSec() > 0.2)
            {
                std_msgs::UInt8MultiArray msg;
                msg.data.assign(stop_cmd, stop_cmd + 5);
                control_pub.publish(msg);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    // 退出前发一次停车
    std_msgs::UInt8MultiArray msg;
    msg.data.assign(stop_cmd, stop_cmd + 5);
    control_pub.publish(msg);

    close(serverSocket);
    if (ser.isOpen()) ser.close();
    return 0;
}