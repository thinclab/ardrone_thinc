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

// ardrone_thinc
#include "ArdroneThinc.hpp"

using namespace cv;
using namespace std;

// threshold images and adjust drone position accordingly
void ArdroneThinc::CamCallback(const sensor_msgs::ImageConstPtr& rosimg) {
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
