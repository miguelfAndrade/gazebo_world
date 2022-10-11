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


//gazebo.msgs.LaserScanStamped

void cb(ConstLaserScanStampedPtr &_msg)
{
    gazebo::msgs::LaserScan scan = _msg->scan();
    gazebo::msgs::Pose world_data = scan.world_pose();
    gazebo::msgs::Vector3d position = world_data.position();
    gazebo::msgs::Quaternion orientation = world_data.orientation();
    std::ofstream file;
    file.open("../sensor_data/data.xml", std::fstream::app); //open is the method of ofstream
    file << "\n"; // << operator which is used to print the file informations in the screen
    file << scan.DebugString();
    // text = text + "\n" + std::to_string(scan.count());
    file.close();
}

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
    
    // std::ofstream o;
    // o.open("../sensor_data/data.txt"); //open is the method of ofstream
    // Publish to the  velodyne topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/my_velodyne/velodyne_hdl-32/top/sensor/scan", cb);
    // o << text;
    // o.close();

    while (true)
        gazebo::common::Time::MSleep(10);

    // Make sure to shut everything down.
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
    #else
    gazebo::client::shutdown();
    #endif
}