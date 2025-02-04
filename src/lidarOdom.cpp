/*
 * @Author: hejia 2736463842@qq.com
 * @Date: 2025-02-04 10:40:04
 * @LastEditors: hejia 2736463842@qq.com
 * @LastEditTime: 2025-02-04 10:41:16
 * @FilePath: /ego-planner-swarm/src/Wheel-Odometry/src/lidarOdom.cpp
 * @Description: 将雷达里程计数据发送给码盘，用于码盘校准
 * Copyright (c) 2025 by hejia 2736463842@qq.com, All Rights Reserved. 
 */
#include "CAN.hpp"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    /* ROS Initialize */
    ros::init(argc, argv, "lidar_odom");
    ros::NodeHandle nh;
    usbCANFD can(nh);

    /* 接收线程 */
    std::thread receiverThread(&usbCANFD::receiveCanMessages, &can);
    while(true){
        if (!can.receiverRunning)
        {
            std::cerr << "Receiver thread not running, restarting..." << std::endl;
            receiverThread.detach();
            can.receiverRunning = true;                                        // 重置标志
            receiverThread = std::thread(&usbCANFD::receiveCanMessages, &can); // 重启线程
        }
    }
    receiverThread.join();

    return 0;
}

