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
    cvtColor(cvimg->image, cvimghsv->image, CV_RGB2HSV);
    inRange(cvimghsv->image,
            Scalar(0, 0, 240),
            Scalar(180, 255, 255),
            cvimgthresh->image);
    cvimgthresh->encoding = "mono8";
    sensor_msgs::ImagePtr rosimgthresh = cvimgthresh->toImageMsg();
    thresh.publish(rosimgthresh);
    Moments m = moments(cvimgthresh->image);
    double x = m.m10/m.m00;
    double y = m.m01/m.m00;
    double xp = x/cvimgthresh->image.size.p[1];
    double yp = y/cvimgthresh->image.size.p[0];
    ROS_INFO("position: (%f, %f)\n", xp, yp);
    
    //// center drone
    //twist_msg.linear.x = 0;
    //twist_msg.linear.y = 0;
    //twist_msg.linear.z = 0;
    //double vel = 0.01;
    //if(xp > 0.7) twist_msg.linear.y = -vel;
    //else if(xp < 0.3) twist_msg.linear.y = vel;
    //if(yp > 0.7) twist_msg.linear.x = -vel;
    //else if(yp < 0.3) twist_msg.linear.x = vel;
    //twist.publish(twist_msg);
}
