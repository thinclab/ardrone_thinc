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
    //if(!centralize) return;

    // convert ros image to opencv image
    cv_bridge::CvImagePtr orig = cv_bridge::toCvCopy(rosimg);
    cv_bridge::CvImagePtr grey(new cv_bridge::CvImage()); 

    // resize image (faster processing);
    int orig_h = orig->image.size.p[0];
    int orig_w = orig->image.size.p[1];
    int new_h = orig_h;
    int new_w = (new_h*orig_w)/orig_h;
    resize(orig->image, orig->image, Size(new_w, new_h), 0, 0);
    Point2d img_center(new_w/2, new_h/2);

    // detect circles using hough transform
    cvtColor(orig->image, grey->image, CV_RGB2GRAY); // rgb -> grey
    GaussianBlur(grey->image, grey->image, Size(3, 3), 0); // denoise
    vector<Vec3f> c;
    HoughCircles(grey->image, c, CV_HOUGH_GRADIENT, 2, 5, 220, 120);
    Point2d avg_center(0, 0);
    for(size_t i = 0; i < c.size(); i++) {
        Point2d center(c[i][0], c[i][1]);
        avg_center += (1.0/(i+1)) * (center - avg_center); // incremental avg
        int radius = cvRound(c[i][2]);
        circle(orig->image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        circle(orig->image, center, radius, Scalar(255, 0, 0), 3, 8, 0);
    }
    circle(orig->image, avg_center, 5, Scalar(0, 0, 255), -1, 8, 0);
    circle(orig->image, img_center, 0.25*min(orig_w, orig_h), Scalar(0, 0, 255));

    // convert opencv image to ros image and publish
    thresh.publish(orig->toImageMsg());
    
    // center drone
    if(avg_center == Point2d(0, 0)) avg_center = img_center;
    Point2d diff = img_center - avg_center;
    if(norm(diff)/min(orig_w, orig_h) > 0.25) {
        twist_msg.linear.x = -diff.y/norm(diff)/4;
        twist_msg.linear.y = diff.x/norm(diff)/4;
        ROS_INFO("x: %f", twist_msg.linear.x);
        ROS_INFO("y: %f", twist_msg.linear.y);
        ROS_INFO("");
    } else {
        twist_msg.linear.x = 0;
        twist_msg.linear.y = 0;
    }
    twist.publish(twist_msg);
}

bool move(ardrone_thinc::Waypoint_Navigator::Request &req, ardrone_thinc::Waypoint_Navigator::Response &res) {
    //check for valid grid cell & look for drone, then move   
    return true;
}



/*
 * Create function to add drones to vector from thinc_main.
 * Give access to number of columns and rows in grid.
 */
