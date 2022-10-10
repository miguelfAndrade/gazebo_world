#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>
#include <fstream>

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

void cb(ConstWorldStatisticsPtr &_msg)
{
    // std::cout << _msg->DebugString();
    std::ofstream o;
    o.open("../data.txt"); //open is the method of ofstream
    o << _msg->DebugString(); // << operator which is used to print the file informations in the screen
    o.close();
}

// void cb(std::string &_msg)
// {
//     // std::cout << _msg->DebugString();
//     std::ofstream o;
//     o.open("../data.txt"); //open is the method of ofstream
//     o << _msg; // << operator which is used to print the file informations in the screen
//     o.close();
// }

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(_argc, _argv);
    #else
    gazebo::client::setup(_argc, _argv);
    #endif

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    
    // Publish to the  velodyne topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/my_velodyne/velodyne_hdl-32/top/sensor/scan", cb);

    while (true)
        gazebo::common::Time::MSleep(10);

    // Make sure to shut everything down.
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
    #else
    gazebo::client::shutdown();
    #endif
}