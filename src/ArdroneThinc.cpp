// ros
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Twist.h"

// opencv2
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

// ardrone_thinc
#include "ArdroneThinc.hpp"

using namespace cv;
using namespace std;

// threshold images and adjust drone position accordingly
void ArdroneThinc::CamCallback(const sensor_msgs::ImageConstPtr& rosimg) {
    cv_bridge::CvImageConstPtr cvimg = cv_bridge::toCvShare(rosimg);
    cv_bridge::CvImagePtr cvimghsv(new cv_bridge::CvImage()); 
    cv_bridge::CvImagePtr cvimgthresh(new cv_bridge::CvImage()); 
    cvtColor(cvimg->image, cvimghsv->image, CV_RGB2YCrCb);
    blur(cvimghsv->image, cvimghsv->image, Size(5, 5));
    inRange(cvimghsv->image,
            Scalar(180, 135, 145),
            Scalar(220, 145, 170),
            cvimgthresh->image);
    cvimgthresh->encoding = "mono8";
    sensor_msgs::ImagePtr rosimgthresh = cvimgthresh->toImageMsg();
    thresh.publish(rosimgthresh);
    Moments m = moments(cvimgthresh->image);
    double x = m.m10/m.m00;
    double y = m.m01/m.m00;
    double xp = x/cvimgthresh->image.size.p[1];
    double yp = y/cvimgthresh->image.size.p[0];
    ROS_INFO("position: (%f, %f)", xp, yp);
    ROS_INFO("area: %f", m.m00);
    
    // center drone
    const double vel = 0.05;
    const double lbound = 0.2;
    const double ubound = 0.8;
    if(xp > ubound) twist_msg.linear.y = -vel;
    else if(xp < lbound) twist_msg.linear.y = vel;
    else if(lbound < xp && xp < ubound) twist_msg.linear.y = 0;
    if(yp > ubound) twist_msg.linear.x = -vel;
    else if(yp < lbound) twist_msg.linear.x = vel;
    else if(lbound < yp && yp < ubound) twist_msg.linear.x = 0;
    ROS_INFO("twist message is: x=%f, y=%f",
            twist_msg.linear.x,
            twist_msg.linear.y);
    //twist.publish(twist_msg);
}
