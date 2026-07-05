#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
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
#include <cmath>        // floor 函数需要
#include <fcntl.h>      // 【新增】用于设置非阻塞模式

using namespace std;

// 保持原有的结构体定义
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

// 全局变量
INFO dc;
uint8_t recv_info_buf[sizeof(INFO)];
// 定义停车指令（全0x0a或根据协议定义的停止位）
uint8_t stop_cmd[5] = {0xff, 0x0a, 0x0a, 0x0a, 0x0a};
uint8_t data[5] = {0xff, 0x0a, 0x0a, 0x0a, 0x0a};

int main(int argc, char** argv)
{
    // 1. ROS 初始化
    ros::init(argc, argv, "my_serial_port_control");
    ros::NodeHandle n;

    // 2. 网络初始化
    struct sockaddr_in Server;
    int serverSocket = socket(PF_INET, SOCK_DGRAM, 0); 
    
    // 【关键优化】：设置 Socket 为非阻塞模式
    // 这样 recvfrom 如果没收到数据会立刻返回 -1，而不是卡死主循环
    int flags = fcntl(serverSocket, F_GETFL, 0);
    fcntl(serverSocket, F_SETFL, flags | O_NONBLOCK);

    Server.sin_family = AF_INET;
    // 使用您指定的固定 IP
    Server.sin_addr.s_addr = inet_addr("192.168.166.49"); 
    Server.sin_port = htons(7950);
    
    // 绑定端口
    if (bind(serverSocket, (sockaddr *)&Server, sizeof(sockaddr)) < 0) {
        ROS_ERROR("Socket Bind Failed! Please check IP address.");
        return -1;
    }

    // 3. 串口初始化
    serial::Serial ser;
    ser.setPort("/dev/ttyUSB0");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(200);
    ser.setTimeout(to);
    
    try {
        ser.open();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Unable to open serial port /dev/ttyUSB0");
        return -1;
    }

    if(ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized.");
    } else {
        return -1;
    }

    // 提高循环频率到 50Hz，让控制更丝滑（原为20Hz）
    ros::Rate loop_rate(100); 
    
    // 用于检测超时的计时器
    ros::Time last_msg_time = ros::Time::now();

    ROS_INFO("Listening for UDP data...");

    // 【关键优化】：使用 ros::ok() 作为循环条件，允许 Ctrl+C 退出
    while(ros::ok()) 
    {
        // 临时变量存储发送方信息，避免覆盖 Server 结构体
        struct sockaddr_in sender_addr;
        socklen_t len = sizeof(sender_addr);
        
        // 非阻塞读取
        int tt = recvfrom(serverSocket, recv_info_buf, sizeof(recv_info_buf), 0, (sockaddr *)&sender_addr, &len);
        
        if(tt > 0) 
        {
            // --- A. 成功收到数据 ---
            last_msg_time = ros::Time::now(); // 喂狗：更新最后收到数据的时间

            memcpy(&dc, recv_info_buf, sizeof(recv_info_buf));
            
            int aa = static_cast<int>(dc.header);
            int bb = static_cast<int>(dc.taitou_12);
            int cc = static_cast<int>(dc.shifang_4);
            int dd = static_cast<int>(dc.qianjin_16);
            int ee = static_cast<int>(dc.zuozhuan_17);

            // 调试打印（可选，如果觉得刷屏太快可以注释掉）
            // cout << "Header: " << aa << " | Data: " << bb << " " << cc << " " << dd << " " << ee << endl;

            if(aa == 255)
            {
                data[0] = aa & 0xff;
                
                // --- 数据解析逻辑 (保持原样) ---
                // data[1] taitou
                if(bb < 110) {
                    bb = 10 - floor(bb/11);
                    data[1] = bb & 0xff;    
                } else if(bb > 130) {
                    bb = 10 + floor((bb - 130)/12.5);
                    data[1] = bb & 0xff;
                } else {
                    data[1] = 0x0a;
                }
                
                // data[2] shifang
                if(cc < 50) {
                    data[2] = 0x00; 
                } else if(cc > 200) {
                    data[2] = 0xff;
                } else {
                    data[2] = 0x0a;
                }
                
                // data[3] qianjin
                if(dd < 110) {
                    dd = floor(1.25*dd -27.5);
                    dd = floor(dd/11);
                    data[3] = dd & 0xff;    
                } else if(dd > 130) {
                    dd = floor(1.53*dd - 63.6);
                    dd = 10 + floor((dd - 120)/12.5);
                    data[3] = dd & 0xff;
                } else {
                    data[3] = 0x0a;
                }
                
                // data[4] zuozhuan
                if(ee < 110) {
                    ee = floor(1.25*ee-27.5);
	   	     ee=floor(ee/11);
                    data[4] = ee & 0xff;    
                } else if(ee > 130) {
	            ee=floor(1.53*ee-63.6);
                    ee = 10 + floor((ee - 120)/12.5);
                    data[4] = ee & 0xff;
                } else {
                    data[4] = 0x0a;
                }
                
                // 发送计算好的数据
                ser.write(data, 5);
            }
            else
            {
                // Header不对，发送停车
               // ser.write(stop_cmd, 5);
            }
        }
        else 
        {
            // --- B. 没收到数据 (tt < 0) ---
            // 【关键优化】：超时保护（看门狗）
            // 如果超过 0.2 秒没收到新指令，强制发送停车指令
            // 这样即使网络断了，机器人也会在 0.2秒内停下，不会失控
            //if ((ros::Time::now() - last_msg_time).toSec() > 0.2) {
              //  ser.write(stop_cmd, 5); 
            //}
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    // 4. 程序退出时的清理工作
    ROS_INFO("Shutting down: stopping robot and closing ports...");
    ser.write(stop_cmd, 5); // 确保退出前发最后一次停车
    usleep(10000);          // 稍微等一下让串口发完
    
    close(serverSocket);
    ser.close();
    
    return 0;
}
