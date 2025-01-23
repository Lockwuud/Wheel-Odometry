/*
 * @Author: hejia 2736463842@qq.com
 * @Date: 2024-12-21 20:44:05
 * @LastEditors: slighty-white 5273495@qq.com
 * @LastEditTime: 2025-01-21 19:57:06
 * @FilePath: /USB-CANFD-Flock/src/CAN.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by hejia 2736463842@qq.com, All Rights Reserved. 
 */

#include "CAN.hpp"

/* 线程池 */

/**
 * @description: 构造函数，初始化线程池
 * @param {size_t} numThreads
 * @return {*}
 */
ThreadPool::ThreadPool(size_t numThreads) : stop(false) {
    for (size_t i = 0; i < numThreads; ++i) {
        workers.push_back(std::thread([this]() { workerLoop(); }));
    }
}

/**
 * @description: 析构函数，停止线程池中的所有线程
 * @return {*}
 */
ThreadPool::~ThreadPool() {
    stop = true;         
    condVar.notify_all();  
    for (std::thread &worker : workers) {
        worker.join();     
    }
}

/**
 * @description: 向线程池队列中添加一个任务
 * @param {function<void()>} task
 * @return {*}
 */
void ThreadPool::enqueue(std::function<void()> task) {
    {
        std::lock_guard<std::mutex> lock(queueMutex); 
        tasks.push(task);  
    }
    condVar.notify_one();
}

/**
 * @description: 线程池工作循环
 * @return {*}
 */
void ThreadPool::workerLoop() {
    while (!stop) 
    { 
        std::function<void()> task;  
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            condVar.wait(lock, [this]() { return stop || !tasks.empty(); });  // 等待任务或停止信号

            if (stop && tasks.empty()) {
                return;
            }

            task = tasks.front();
            tasks.pop(); 
        }

        task();
    }
}



/* 文件锁 */

/**
 * @description: 构造函数，初始化读写锁、CAN套接字
 * @return {*}
 */
fileLock::fileLock(){
    /* 写锁初始化 */
    lock_write.l_whence = SEEK_SET;   // 从文件开始位置加锁
    lock_write.l_start = 0;           // 锁定范围起始位置
    lock_write.l_len = 0;             // 锁定整个文件
    
    /* 读锁初始化 */
    lock_read.l_whence = SEEK_SET;
    lock_read.l_start = 0;
    lock_read.l_len = 0;
}

fileLock::~fileLock(){

}

/**
 * @description: 加写锁
 * @return {*}
 */
void fileLock::lock_w(){
    lock_write.l_type = F_WRLCK;      // 独占锁（写锁）
    fcntl(socket_can, F_SETLKW, &lock_write);
}

/**
 * @description: 开写锁
 * @return {*}
 */
void fileLock::unlock_w(){
    lock_write.l_type = F_UNLCK;
    fcntl(socket_can, F_SETLK, &lock_write);
}

/**
 * @description: 加读锁
 * @return {*}
 */
void fileLock::lock_r(){
    lock_read.l_type = F_RDLCK;     
    fcntl(socket_can, F_SETLKW, &lock_read);
}

/**
 * @description: 开读锁
 * @return {*}
 */
void fileLock::unlock_r(){
    lock_read.l_type = F_UNLCK;
    fcntl(socket_can, F_SETLK, &lock_read);
}



/* CAN 发送 */

/**
 * @description: 构造函数，初始化can编号
 * @param {string} &canNAME
 * @return {*}
 */
usbCANFD::usbCANFD(ros::NodeHandle &n) : receiverRunning(true),
                                         threadPool(thread_nums),
                                         stopReceiving(false),
                                         nh(n){
    pub = nh.advertise<nav_msgs::Odometry>("/wheel_odom", 10);
    initialize();
}

/**
 * @description: 析构函数，关闭 CAN 套接字
 * @return {*}
 */
usbCANFD::~usbCANFD() 
{
    if (sock >= 0) {
        close(sock);  
    }
}

/**
 * @description: 初始化 CAN 通信
 * @return {*}
 */
bool usbCANFD::initialize()
{
    /* 初始化can设备 */
    int sta_1 = system(ip_cmd_can0_down);
    int sta_2 = system(ip_cmd_set_can0_params);
    int sta_3 = system(ip_cmd_can0_up);

    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {  
        perror("Error while opening socket");
        return false;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, interfaceName);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("Error while getting interface index");
        return false;
    }

    struct sockaddr_can addr;  
    addr.can_family = AF_CAN;  
    addr.can_ifindex = ifr.ifr_ifindex;  

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) { 
        perror("Error while binding socket");
        return false;
    }

    int enable_canfd = 1;
    if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) != 0) {
    perror("Failed to enable CAN FD");
    return false;
    }

    lock.socket_can = sock;    
    return true;
}

/**
 * @description: 发送 CAN 消息
 * @return {*}
 */
void usbCANFD::sendCanMessage() 
{
    lock.lock_w();
    if (write(sock, &sendframe, sizeof(sendframe)) != sizeof(sendframe)) {
        lock.unlock_w();
        printf("Error while sending CAN message\n");
        printf("Reconnecting......\n");
        if (sock >= 0) {
            close(sock);
            sock = -1;
        }
        if(!initialize()){
            printf("Reconnect failed, give up this sending!!!\n");
            receiverRunning = false;
            return;
        }
        else{
            printf("Reconnect successfully!!!\n");
            lock.lock_w();
            if(write(sock, &sendframe, sizeof(sendframe)) != sizeof(sendframe)){
                lock.unlock_w();
                receiverRunning = false;
                printf("Error while sending message after reconnection\n");
                return;
            }
        }
    }
    lock.unlock_w();
    // printf("Send successfully!!!\n");
}

/**
 * @description: 接收 CAN 消息
 * @param {ThreadPool} &threadPool
 * @return {*}
 */
void usbCANFD::receiveCanMessages() 
{
    while (!stopReceiving) {  
        /* 接收 CAN 帧 */
        
        canfd_frame frame;  
        lock.lock_r();
        int nbytes = read(sock, &frame, sizeof(frame));  
        lock.unlock_r();
        if (nbytes < 0) {  
            perror("Error while receiving CAN message");
            continue;
        }

        /* 解析 CAN 消息 */
        threadPool.enqueue([this, frame]() {
            this->parseCanMessage(frame); 
        });
    }
}

/**
 * @description: 停止接收数据
 * @return {*}
 */
void usbCANFD::stopReceivingData() {
    stopReceiving = true;
}

/**
 * @description: 数据解包
 * @param {can_frame} &frame
 * @return {*}
 */
void usbCANFD::parseCanMessage(const canfd_frame &frame) 
{
    std::lock_guard<std::mutex> lock(parseMutex); 
    std::cout << "Received CAN ID: " << frame.can_id << std::endl;  
    std::cout << "Data: ";
    for (int i = 0; i < frame.len; ++i) { 
        std::cout << std::hex << (int)frame.data[i] << " "; 
    }
    std::cout << std::endl;

    switch (frame.can_id)
    {
    case receive_id_1:
        customReceive_1(frame);
        break;
    
    case receive_id_2:
        customReceive_2(frame);
        break;

    default:
        break;
    }
}

/**
 * @description: 自定义接收例程 1
 * @return {*}
 */
void usbCANFD::customReceive_1(const canfd_frame &frame)
{
    receiveData.clear();
    float yaw, x, y, vx, vy;
    memcpy(&x, &frame.data[0], 4);
    memcpy(&y, &frame.data[4], 4);
    memcpy(&vx, &frame.data[8], 4);
    memcpy(&vy, &frame.data[12], 4);
    memcpy(&yaw, &frame.data[16], 4);

    /* 发布里程计 */
    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp = ros::Time::now(); // 当前时间戳
    odom_msg.header.frame_id = "odom";        // 参考坐标系
    odom_msg.child_frame_id = "base_link";    // 子坐标系

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = yaw; 

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = 0.0; 

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0; 

    pub.publish(odom_msg);

    /* 发布动态坐标转换 */
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;

    tfs.header.frame_id = "odom";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "pcd_frame";

    tfs.transform.translation.x = x;
    tfs.transform.translation.y = y;
    tfs.transform.translation.z = yaw;

    tfs.transform.rotation.x = 0;
    tfs.transform.rotation.y = 0;
    tfs.transform.rotation.z = 0;
    tfs.transform.rotation.w = 1.0;

    broadcaster.sendTransform(tfs);
}

/**
 * @description: 自定义接收例程 2
 * @return {*}
 */
void usbCANFD::customReceive_2(const canfd_frame &frame)
{
    memcpy(&order, &frame.data[0], 1);
}

/**
 * @description: 获取can时钟
 * @return {*}
 */
uint32_t usbCANFD::getTimeSyne()
{
    while (true)
    {
        /* 接收 CAN 帧 */
        can_frame frame;
        lock.lock_r();
        int nbytes = read(sock, &frame, sizeof(frame));
        lock.unlock_r();
        if (nbytes < 0)
        {
            perror("Error while receiving CAN message");
            continue;
        }

        /* 解析 CAN 消息 */
        if (frame.can_id == 0x666)
        {
            uint32_t time = -1;
            memcpy(&time, &frame.data[0], 4);
            printf("Now time is %d\n", time);
            return time;
        }
        else continue;
    }
}
