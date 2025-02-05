/*
 * @Author: hejia 2736463842@qq.com
 * @Date: 2024-12-22 01:17:56
 * @LastEditors: hejia 2736463842@qq.com
 * @LastEditTime: 2025-02-04 11:38:02
 * @FilePath: /ego-planner-swarm/src/Wheel-Odometry/include/CAN.hpp
 * @Description:
 *
 * Copyright (c) 2024 by hejia 2736463842@qq.com, All Rights Reserved.
 */

#ifndef CAN_HPP
#define CAN_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <iostream>           // 包含输入输出流，用于打印日志或调试信息
#include <string>             // 包含 string 类的定义，用于字符串操作
#include <cstring>            // 包含 C 风格字符串操作函数的定义
#include <unistd.h>           // 包含 POSIX 系统调用，例如 close, read, write
#include <net/if.h>           // 包含网络接口相关的定义
#include <sys/ioctl.h>        // 包含 I/O 控制相关的函数定义，例如 SIOCGIFINDEX
#include <fcntl.h>            // 包含文件控制相关的定义
#include <linux/can.h>        // 包含 CAN 协议相关的定义
#include <linux/can/raw.h>    // 包含原始 CAN 套接字定义
#include <sys/socket.h>       // 包含 socket 套接字的相关定义
#include <thread>             // 包含多线程支持的定义
#include <atomic>             // 包含原子操作的定义，用于线程安全
#include <mutex>              // 包含互斥量的定义，用于线程同步
#include <vector>             // 包含 vector 容器的定义
#include <queue>              // 包含队列容器的定义
#include <functional>         // 包含函数对象的定义，用于队列任务
#include <condition_variable> // 包含条件变量定义，用于线程同步
#include <chrono>             // 包含时间相关定义，用于控制线程等待时间
#include <iostream>           // 包含 I/O 相关功能

// 连接系统命令echo "your_password" | sudo -S command_to_run
#define ip_cmd_set_can0_params "echo 6 | sudo -S ip link set can0 type can bitrate 1000000 dbitrate 2000000 fd on"
#define ip_cmd_can0_up "echo 6 | sudo -S ip link set can0 up"
#define ip_cmd_can0_down "echo 6 | sudo -S ip link set can0 down"

// 异步接收线程池大小
#define thread_nums 4

// 设备名称
#define interfaceName "can0"

// CAN_ID
#define send_id_lidar_odom 0x188
#define receive_id_1 0x666
#define receive_id_2 0x001
#define receive_id_time 0x168

class ThreadPool
{
private:
    // 线程池中的工作线程函数
    void workerLoop();

    std::vector<std::thread> workers;        // 线程池中的所有线程
    std::queue<std::function<void()>> tasks; // 任务队列，存储待处理的任务
    std::mutex queueMutex;                   // 互斥锁，用于保护任务队列
    std::condition_variable condVar;         // 条件变量，用于通知线程执行任务
    std::atomic<bool> stop;                  // 原子变量，用于控制线程池的停止

public:
    ThreadPool(size_t numThreads);

    ~ThreadPool();

    void enqueue(std::function<void()> task);
};

class fileLock
{
private:
    struct flock lock_write;
    struct flock lock_read;

public:
    int socket_can;
    fileLock();
    ~fileLock();
    void lock_w();
    void unlock_w();
    void lock_r();
    void unlock_r();
};

class usbCANFD
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    int sock;                        // CAN 套接字
    std::atomic<bool> stopReceiving; // 接收线程池停止信号
    std::mutex parseMutex;           // 解包互斥锁
    std::mutex sendframe_mutex;      // 发送互斥锁
    fileLock lock;                   // 文件锁
    ThreadPool threadPool;           // 异步接收线程池
    canfd_frame sendframe;           // 发送帧

    template <typename T>
    void handleData(const std::vector<T> &data);
    void parseCanMessage(const canfd_frame &frame);

public:
    std::atomic<bool> receiverRunning; // 用于监控接收线程状态
    std::vector<float> receiveData;    // 解析数据寄存器
    uint8_t order;                     // 解析数据寄存器

    usbCANFD(ros::NodeHandle &n);

    ~usbCANFD();

    bool initialize();

    void sendCanMessage();

    void receiveCanMessages();

    void stopReceivingData();

    uint32_t getTimeSyne();

    void customReceive_1(const canfd_frame &frame);

    void customReceive_2(const canfd_frame &frame);

    template <typename T>
    void sendData(uint32_t send_id, const std::vector<T> &data, int dlcparam);

    void lidar_odom_cbk(const nav_msgs::Odometry::ConstPtr &msg);

    void sendLidarOdom();
};

/**
 * @description: 数据类处理模板
 * @return {*}
 */
template <typename T>
void usbCANFD::handleData(const std::vector<T> &data)
{
    size_t dataSize = data.size() * sizeof(T);
    if (dataSize > sizeof(sendframe.data))
    {
        std::cerr << "Data size exceeds frame buffer size!" << std::endl;
        return;
    }
    memcpy(sendframe.data, data.data(), dataSize);
}

/**
 * @description: 发送端模板函数
 * @param {uint32_t} can_id
 * @param {dataType} type
 * @param {DLC} dlcparam
 * @return {*}
 */
template <typename T>
void usbCANFD::sendData(uint32_t send_id, const std::vector<T> &data, int dlcparam)
{
    std::lock_guard<std::mutex> lock(sendframe_mutex);
    memset(&sendframe, 0, sizeof(sendframe));
    sendframe.can_id = send_id;
    sendframe.len = dlcparam;
    sendframe.flags = 0x01;
    handleData<T>(data);
    sendCanMessage();
}

#endif
