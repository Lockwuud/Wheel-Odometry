#include "CAN.hpp"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    /* ROS Initialize */
    ros::init(argc, argv, "wheel_odom");
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

