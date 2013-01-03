// ros
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"

// opencv2
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

// ardrone_autonomy
#include "ardrone_autonomy/CamChannel.h"

using namespace cv;
using namespace std;

// threshold images and adjust drone position accordingly
void CamCallback(const sensor_msgs::ImageConstPtr& rosimg) {
    cv_bridge::CvImageConstPtr cvimg = cv_bridge::toCvShare(rosimg);
    Mat cvimghsv; Mat cvimgthresh;
    cvtColor(cvimg->image, cvimghsv, CV_RGB2HSV);
    inRange(cvimghsv,
            Scalar(0, 0, 240),
            Scalar(180, 255, 255),
            cvimgthresh);
    Moments m = moments(cvimgthresh);
    double x = m.m10/m.m00;
    double y = m.m01/m.m00;
    double xp = x/cvimgthresh.size.p[1];
    double yp = y/cvimgthresh.size.p[0];
    ROS_INFO("position: (%f, %f)\n", xp, yp);
}

int main(int argc, char **argv) {
    // ros init
    ros::init(argc, argv, "thinc_main");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // publisher, subscribers, and services
    ros::Publisher launch = n.advertise<std_msgs::Empty>("ardrone/takeoff", 5);
    ros::Publisher land = n.advertise<std_msgs::Empty>("ardrone/land", 5);
    ros::Publisher reset = n.advertise<std_msgs::Empty>("ardrone/reset", 5);
    ros::Publisher twist = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber cam =
        n.subscribe<sensor_msgs::Image>("ardrone/image_raw", 1, CamCallback);
    ros::ServiceClient camchannel =
        n.serviceClient<ardrone_autonomy::CamChannel>("ardrone/setcamchannel");

    // messages for takeoff and landing
    std_msgs::Empty empty_msg;
    std_msgs::Twist twist_msg;

    // set camchannel on drone and takeoff
    if(ros::ok()) {
        ardrone_autonomy::CamChannel camsrv;
        camsrv.request.channel = 1;
        launch.pub(empty_msg);
    } 
    
    // hover drone in place
    while(ros::ok()) {
        ros::spinOnce(); // see CamCallback()
        loop_rate.sleep();
    }

    return 0;
}
