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

#define VEL 0.05
#define LB 0.2
#define UB 0.8

// threshold images and adjust drone position accordingly
void ArdroneThinc::CamCallback(const sensor_msgs::ImageConstPtr& rosimg) {
    cv_bridge::CvImageConstPtr cvimg = cv_bridge::toCvShare(rosimg);
    // convert ros image to opencv image
    cv_bridge::CvImagePtr cvimghsv(new cv_bridge::CvImage()); 
    cv_bridge::CvImagePtr cvimgthresh(new cv_bridge::CvImage()); 
    cvtColor(cvimg->image, cvimghsv->image, CV_RGB2HSV); // RGB->HSV
    blur(cvimghsv->image, cvimghsv->image, Size(2, 2)); // remove noise
    // threshold image
    inRange(cvimghsv->image,
            Scalar(140, 0, 200),
            Scalar(180, 255, 255),
            cvimgthresh->image);
    // calculate center of white pixels in thresholded image
    Moments m = moments(cvimgthresh->image);
    int imgw = cvimgthresh->image.size.p[1];
    int imgh = cvimgthresh->image.size.p[0];
    double x = m.m10/m.m00;
    double y = m.m01/m.m00;
    double xp = x/imgw;
    double yp = y/imgh;
    // mark center of threshold and image, and draw boundary lines
    Scalar mc(128); // marking color (mono, anyway)
    circle(cvimgthresh->image, Point(x, y), 1, mc, -1);                   
    Point center(imgw/2, imgh/2);
    circle(cvimgthresh->image, center, 1, mc, -1);                   
    line(cvimgthresh->image, Point(0, imgh*LB), Point(imgw, imgh*LB), mc);
    line(cvimgthresh->image, Point(0, imgh*UB), Point(imgw, imgh*UB), mc);
    line(cvimgthresh->image, Point(imgw*LB, 0), Point(imgw*LB, imgh), mc);
    line(cvimgthresh->image, Point(imgw*UB, 0), Point(imgw*UB, imgh), mc);

    // convert opencv image to ros image and publish
    cvimgthresh->encoding = "mono8";
    sensor_msgs::ImagePtr rosimgthresh = cvimgthresh->toImageMsg();
    thresh.publish(rosimgthresh);
    
    // center drone
    if(xp > UB) twist_msg.linear.y = -VEL;
    else if(xp < LB) twist_msg.linear.y = VEL;
    else if(LB < xp && xp < UB) twist_msg.linear.y = 0;
    if(yp > UB) twist_msg.linear.x = -VEL;
    else if(yp < LB) twist_msg.linear.x = VEL;
    else if(LB < yp && yp < UB) twist_msg.linear.x = 0;
    twist.publish(twist_msg);
}
