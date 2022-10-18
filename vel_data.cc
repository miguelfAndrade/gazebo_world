#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>
#include <fstream>

using namespace std;

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif


string body_tags = ""; //all repeated tags are added in this global string variable

int counter = 0;

void cb(ConstLaserScanStampedPtr &_msg)
{
    counter += 1;
    int i = 0;

    string xmlheader = "<?xml version=\"1.0\" enconding=\"utf-8\"?>\n<data>\n"; //xml tag header and data tag
    string xml_final_tag = "</data>"; //xml end data tag
    
    gazebo::msgs::LaserScan scan = _msg->scan();
    gazebo::msgs::Pose world_data = scan.world_pose();
    gazebo::msgs::Vector3d position = world_data.position();
    gazebo::msgs::Quaternion orientation = world_data.orientation();
    
    int ray_total_count = scan.count();

    string x_pos_tag = "\t\t\t\t<x>" + to_string(position.x()) + "</x>\n";
    string y_pos_tag = "\t\t\t\t<y>" + to_string(position.y()) + "</y>\n";
    string z_pos_tag = "\t\t\t\t<z>" + to_string(position.z()) + "</z>\n";

    string x_rot_tag = "\t\t\t\t<x>" + to_string(orientation.x()) + "</x>\n";
    string y_rot_tag = "\t\t\t\t<y>" + to_string(orientation.y()) + "</y>\n";
    string z_rot_tag = "\t\t\t\t<z>" + to_string(orientation.z()) + "</z>\n";
    string w_rot_tag = "\t\t\t\t<w>" + to_string(orientation.w()) + "</w>\n";

    string pos_tag = "\t\t\t<position>\n" + x_pos_tag + y_pos_tag + z_pos_tag + "\t\t\t</position>\n";
    string orientation_tag = "\t\t\t<orientation>\n" + x_rot_tag + y_rot_tag + z_rot_tag + w_rot_tag + "\t\t\t</orientation>\n";

    string world_pose_tag = "\t\t<world_pose>\n" + pos_tag + orientation_tag +"\t\t</world_pose>\n";

    string angle_min_tag = "\t\t<angle_min>" + to_string(scan.angle_min()) + "</angle_min>\n";
    string angle_max_tag = "\t\t<angle_max>" + to_string(scan.angle_max()) + "</angle_max>\n";
    string angle_step_tag = "\t\t<angle_step>" + to_string(scan.angle_step()) + "</angle_step>\n";
    string range_min_tag = "\t\t<range_min>" + to_string(scan.range_min()) + "</range_min>\n";
    string range_max_tag = "\t\t<range_max>" + to_string(scan.range_max()) + "</range_max>\n";
    string count_tag = "\t\t<count>" + to_string(ray_total_count) + "</count>\n";
    string vertical_angle_min_tag = "\t\t<vertical_angle_min>" + to_string(scan.vertical_angle_min()) + "</vertical_angle_min>\n";
    string vertical_angle_max_tag = "\t\t<vertical_angle_max>" + to_string(scan.vertical_angle_max()) + "</vertical_angle_max>\n";
    string vertical_angle_step_tag = "\t\t<vertical_angle_step>" + to_string(scan.vertical_angle_step()) + "</vertical_angle_step>\n";
    string vertical_count_tag = "\t\t<vertical_count>" + to_string(scan.vertical_count()) + "</vertical_count>\n";

    string angle_max_min_tags = angle_min_tag + angle_max_tag + angle_step_tag + range_min_tag + range_max_tag + count_tag + vertical_angle_min_tag + vertical_angle_max_tag + vertical_angle_step_tag + vertical_count_tag;

    string ranges_tags = "";
    string intensities_tags = "";

    while(i<ray_total_count)
    {
        ranges_tags += "\t\t<ranges>" + to_string(scan.ranges(i)) + "</ranges>\n";
        intensities_tags += "\t\t<intensities>" + to_string(scan.intensities(i)) + "</intensities>\n";
        i += 1;
    }

    string point_tag = "\t<point>\n" + world_pose_tag + angle_max_min_tags + ranges_tags + intensities_tags + "\t</point>\n";
    
    body_tags += point_tag;

    ofstream file;
    // file.open("../sensor_data/sensor_data.xml", fstream::app);
    file.open("../sensor_data/sensor_data.xml"); //open is the method of ofstream
    file << xmlheader + body_tags + xml_final_tag;
    file.close();
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{

    // if(isdigit(to_string(_argv[1])) == false)
    // {
    //     cout << "Please enter a valid number as an argument for max_counter variable!!";
    //     return 0;
    // }
    // int max_counter = (int) _argv[1];
    int max_counter = 200;
    
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

    while (counter < max_counter)
        gazebo::common::Time::MSleep(10);

    // Make sure to shut everything down.
    #if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
    #else
    gazebo::client::shutdown();
    #endif
}