#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"

class ArdroneThinc {
    public:
        ros::Publisher launch;
        ros::Publisher land;
        ros::Publisher reset;
        ros::Publisher twist;
        ros::Subscriber cam;
        ros::ServiceClient camchannel;
        std_msgs::Empty empty_msg;
        geometry_msgs::Twist twist_msg;
        void CamCallback(const sensor_msgs::ImageConstPtr& rosimg);
};
