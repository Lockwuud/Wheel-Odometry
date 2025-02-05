/*
 * @Author: hejia 2736463842@qq.com
 * @Date: 2025-02-04 09:12:11
 * @LastEditors: hejia 2736463842@qq.com
 * @LastEditTime: 2025-02-04 11:40:44
 * @FilePath: /ego-planner-swarm/src/Wheel-Odometry/src/wheelOdom.cpp
 * @Description: 码盘里程计发布及雷达里程计校准码盘
 * Copyright (c) 2025 by hejia 2736463842@qq.com, All Rights Reserved. 
 */
#include "CAN.hpp"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    /* ROS Initialize */
    ros::init(argc, argv, "wheel_odom");
    ros::NodeHandle nh;
    usbCANFD can(nh);

    // /* 发送线程 */
    // std::thread sendThread(&usbCANFD::sendLidarOdom, &can);

    /* 接收线程 */
    std::thread receiverThread(&usbCANFD::receiveCanMessages, &can);

    /* 维护线程 */
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
    // sendThread.join();

    return 0;
}

