#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"

#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <cstring>

// using namespace std;

ros::Publisher marker_pub;

// std::map<std::string, std::string> mappify1(std::string const& s)
// {
//     std::map<std::string, std::string> m;

//     std::string key, val;
//     std::istringstream iss(s);

//     while(std::getline(std::getline(iss, key, ':') >> std::ws, val))
//         m[key] = val;

//     return m;
// }


// std::map<std::string, std::string> mappify2(std::string const& s)
// {
//     std::map<std::string, std::string> m;

//     std::string::size_type key_pos = 0;
//     std::string::size_type key_end;
//     std::string::size_type val_pos;
//     std::string::size_type val_end;

//     while((key_end = s.find(':', key_pos)) != std::string::npos)
//     {
//         if((val_pos = s.find_first_not_of(": ", key_end)) == std::string::npos)
//             break;

//         val_end = s.find('\n', val_pos);
//         m.emplace(s.substr(key_pos, key_end - key_pos), s.substr(val_pos, val_end - val_pos));

//         key_pos = val_end;
//         if(key_pos != std::string::npos)
//             ++key_pos;
//     }

//     return m;
// }



void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    // ROS_INFO("I heard: [%s]", msg->data.c_str()); //msg->data.c_str()

    // std::map<std::string, std::string> m = mappify1(msg->data.c_str());
    // for(auto const& p: m)
    //     std::cout << '{' << p.first << " => " << p.second << '}' << '\n';

    // std::string s = msg->data.c_str();
    // char cstr[s.size() + 1];
    // strcpy(cstr, s.c_str());    // or, pass `&s[0]`
 
    // std::cout << cstr << std::endl;


    uint32_t shape = visualization_msgs::Marker::CUBE;

    while (ros::ok())
  {
    /////////////////// START of marker 1 (SPHERE shape) ///////////////////
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one

    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    // marker.pose.position.x = 0;
    // marker.pose.position.y = 0;
    // marker.pose.position.z = 0;

    marker.pose.position = msg->data.pose;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);

    shape = visualization_msgs::Marker::SPHERE;

    /////////////////// END of marker 1 and START of marker 2 (text)  ///////////////////


    visualization_msgs::Marker marker2;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker2.header.frame_id = "/base_link";
    marker2.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker2.ns = "basic_shapes";
    marker2.id = 1;
    marker2.type = shape2;

    marker2.action = visualization_msgs::Marker::ADD;

    marker2.pose.position = marker.pose.position;
    marker2.pose.position.z = marker.pose.position.z + 1;

    marker2.pose.orientation.x = 0.0;
    marker2.pose.orientation.y = 0.0;
    marker2.pose.orientation.z = 0.0;
    marker2.pose.orientation.w = 1.0;

    marker2.scale.x = 1.0;
    marker2.scale.y = 1.0;
    marker2.scale.z = 1.0;

    marker2.color.r = 0.0f;
    marker2.color.g = 1.0f;
    marker2.color.b = 0.0f;
    marker2.color.a = 1.0;

    marker2.lifetime = ros::Duration();

    marker2.text = "DETECTED " + msg->data.num_of_detections + " times";
    
    marker_pub.publish(marker2);
    shape2 = visualization_msgs::Marker::TEXT_VIEW_FACING;

/////////////////// END of marker 2 ///////////////////
  }

}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;

    ros::Rate r(1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber sub = n.subscribe("object_detection/counter", 1000, chatterCallback);

    ros::spin();

    return 0;
}

