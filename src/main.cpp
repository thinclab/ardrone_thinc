#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "thinc_main");
    ros::NodeHandle n;
    ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 5, true);
    ros::Publisher land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 5, true);
    ros::Publisher reset_pub = n.advertise<std_msgs::Empty>("ardrone/reset", 5, true);
    ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10, true);
    ros::Rate loop_rate(10);

//  ros::Duration(1.0).sleep();
    
    if(ros::ok()) {
        std_msgs::Empty takeoff_msg;
        std_msgs::Empty land_msg;
        geometry_msgs::Twist twist_msg;

        ROS_INFO("Theoretically, takeoff.\n");
        takeoff_pub.publish(takeoff_msg);
    
        ros::Duration(15.0).sleep();
        twist_msg.angular.z = 0.5;
        twist_pub.publish(twist_msg);
        ROS_INFO("Theoretically, spinning.\n");
        ros::Duration(5.0).sleep();
        twist_msg.angular.z = 0;
        twist_pub.publish(twist_msg);
        ROS_INFO("Theoretically, hovering.\n");
        ros::Duration(5.0).sleep();

        land_pub.publish(land_msg);
        ROS_INFO("Theoretically, landing.\n");
    }

    return 0;
}
