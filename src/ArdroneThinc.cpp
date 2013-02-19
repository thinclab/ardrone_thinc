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
    // convert ros image to opencv image
    cv_bridge::CvImagePtr orig = cv_bridge::toCvCopy(rosimg);
    cv_bridge::CvImagePtr grey(new cv_bridge::CvImage()); 

    // resize image (faster processing);
    int new_w = orig->image.size.p[0];
    int new_h = (new_w*orig->image.size.p[1])/orig->image.size.p[0];
    resize(orig->image, orig->image, Size(new_w, new_h), 0, 0);

    // detect circles using hough transform
    cvtColor(orig->image, grey->image, CV_RGB2GRAY); // rgb -> grey
    GaussianBlur(grey->image, grey->image, Size(2, 2), 0); // denoise
    vector<Vec3f> c;
    HoughCircles(grey->image, c, CV_HOUGH_GRADIENT, 2, 0, 200, 100);
    for(size_t i = 0; i < c.size(); i++) {
        Point center(cvRound(c[i][0]), cvRound(c[i][1]));
        int radius = cvRound(c[i][2]);
        circle(orig->image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        circle(orig->image, center, radius, Scalar(255, 0, 0), 3, 8, 0);
    }

    // convert opencv image to ros image and publish
    thresh.publish(orig->toImageMsg());
    
    // center drone
    //if(xp > UB) twist_msg.linear.y = -VEL;
    //else if(xp < LB) twist_msg.linear.y = VEL;
    //else if(LB < xp && xp < UB) twist_msg.linear.y = 0;
    //if(yp > UB) twist_msg.linear.x = -VEL;
    //else if(yp < LB) twist_msg.linear.x = VEL;
    //else if(LB < yp && yp < UB) twist_msg.linear.x = 0;
    //twist.publish(twist_msg);
}
