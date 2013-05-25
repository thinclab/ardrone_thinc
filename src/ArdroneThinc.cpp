// ros
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"

// opencv2
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

// ardrone_thinc
#include "ArdroneThinc.hpp"

// misc
#include <cmath>

// degree to radian helper macro
#define D2R(a) (a*M_PI/180)

using namespace cv;

using std::cout;
using std::endl;
using sensor_msgs::ImageConstPtr;
using ardrone_autonomy::NavdataConstPtr;
using ardrone_thinc::Waypoint;

#define MOVE_VEL 0.25

// threshold images and adjust drone position accordingly
void ArdroneThinc::CamCallback(const sensor_msgs::ImageConstPtr& rosimg) {

    // convert ros image to opencv image
    cv_bridge::CvImagePtr orig = cv_bridge::toCvCopy(rosimg);
    cv_bridge::CvImagePtr grey(new cv_bridge::CvImage());

    // resize image (faster processing);
    int new_h = orig->image.size.p[0];
    int new_w = (new_h*orig->image.size.p[1])/orig->image.size.p[0];
    resize(orig->image, orig->image, Size(new_w, new_h), 0, 0);

    // detect circles using hough transform
    cvtColor(orig->image, grey->image, CV_RGB2GRAY); // rgb -> grey
    GaussianBlur(grey->image, grey->image, Size(3, 3), 2, 2); // denoise
    vector<cv::Vec3f> c;

    HoughCircles(grey->image, c, CV_HOUGH_GRADIENT, 2, 2, 100, 50); //220 120

// HoughCircles(grey->image, c, CV_HOUGH_GRADIENT, 2, 2, 150, 30); //220 120

    img_vec = c;
    Point avg_center; // grab from laptop code
    for(size_t i = 0; i < c.size(); i++) {
        Point center(cvRound(c[i][0]), cvRound(c[i][1]));
        int radius = cvRound(c[i][2]);
        avg_center += (Point(c[i][0], c[i][1]) - avg_center)*(1.0/(i+1));
        circle(orig->image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        circle(orig->image, center, radius, Scalar(0, 0, 255), 3, 8, 0);
    }

    double height = sonar/sqrt(1+tan(D2R(rotx))*tan(D2R(rotx))+tan(D2R(roty))*tan(D2R(roty)));
    Point over(height*sin(D2R(rotx)), height*sin(D2R(roty)));
// cout << "height: " << height << endl;
// cout << "over: " << over << endl;

    // convert opencv image to ros image and publish
    thresh_pub.publish(orig->toImageMsg());

/*
    // not implemented yet
    //if(!stabilize) return;

    // convert ros image to opencv image
    cv_bridge::CvImagePtr orig = cv_bridge::toCvCopy(rosimg);
    cv_bridge::CvImagePtr grey(new cv_bridge::CvImage()); 

    // resize image (faster processing);
    int new_h = orig->image.size.p[0];
    int new_w = (new_h*orig->image.size.p[1])/orig->image.size.p[0];
    resize(orig->image, orig->image, Size(new_w, new_h), 0, 0);

    // detect circles using hough transform
    cvtColor(orig->image, grey->image, CV_RGB2GRAY); // rgb -> grey
    GaussianBlur(grey->image, grey->image, Size(3, 3), 2, 2); // denoise
    HoughCircles(grey->image, this->circles, CV_HOUGH_GRADIENT, 2, 2, 220, 120);

    Point avg_center; // grab from laptop code
    for(size_t i = 0; i < this->circles.size(); i++) {
        Point center(cvRound(this->circles[i][0]), cvRound(this->circles[i][1]));
        int radius = cvRound(this->circles[i][2]);
        avg_center += (Point(this->circles[i][0], this->circles[i][1]) - avg_center)*(1.0/(i+1));
        circle(orig->image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        circle(orig->image, center, radius, Scalar(0, 0, 255), 3, 8, 0);
    }

    double tan2x = tan(D2R(this->rotx))*tan(D2R(this->rotx));
    double tan2y = tan(D2R(this->roty))*tan(D2R(this->roty));
    double height = this->sonar/sqrt(1+tan2x+tan2y);

    Point over(height*sin(D2R(rotx)), height*sin(D2R(roty)));
//    cout << "height: " << height << endl;
//    cout << "over: " << over << endl;

    const double WFOV = 58;
    const double HFOV = 47;
    double w_angle = WFOV/2*(2*avg_center.x/new_w - 1);
    double h_angle = HFOV/2*(2*avg_center.y/new_h - 1);
    Point avg_center_mm(height*tan(w_angle), height*tan(h_angle));
    Point move = over - avg_center_mm;

//    cout << "move: " << move << endl;

    // convert opencv image to ros image and publish
    this->thresh_pub.publish(orig->toImageMsg());
    
    // center drone
    //if(xp > UB) twist_msg.linear.y = -VEL;
    //else if(xp < LB) twist_msg.linear.y = VEL;
    //else if(LB < xp && xp < UB) twist_msg.linear.y = 0;
    //if(yp > UB) twist_msg.linear.x = -VEL;
    //else if(yp < LB) twist_msg.linear.x = VEL;
    //else if(LB < yp && yp < UB) twist_msg.linear.x = 0;
    //twist.publish(twist_msg);
*/

}

// collect navdata
void ArdroneThinc::NavdataCallback(const NavdataConstPtr& nav) {
    this->rotx = nav->rotX;
    this->roty = nav->rotY;
    this->sonar = nav->altd;
}

// move to designated sector
bool ArdroneThinc::WaypointCallback(Waypoint::Request &req, Waypoint::Response &res) {
    cout << "waypoint callback!" << endl; 

    // ensure valid grid cell
    if(req.x < 0 || req.y < 0 || req.x >= this->columns || req.y >= this->rows) {
        return false; 
    }

    // deltas
    int dx = this->x - req.x;
    int dy = this->y - req.y;

    // move: x first, then y
    while(ros::ok() && (dx || dy)) {
        if(dx > 0) { 
            move(LEFT); dx--; 
        } else if(dx < 0) { 
            move(RIGHT); dx++;
        } else if(dy < 0) { 
            move(UP); dy++; 
        } else if(dy > 0) {
            move(DOWN); dy--;
        }
    }

    res.x = this->x;
    res.y = this->y;
    res.z = 0;
    return true;
}

// move in the direction given
void ArdroneThinc::move(enum dir d) {

    /* -linear.x: move backward
     * +linear.x: move forward
     * -linear.y: move right
     * +linear.y: move left
     */

    cout << "move" << endl; 

    switch(d) {
        case LEFT: 
            this->twist_msg.linear.y = MOVE_VEL; 
            this->x--;
            break; 
        case RIGHT: 
            this->twist_msg.linear.y = -MOVE_VEL;
            this->x++;
            break; 
        case UP: 
            this->twist_msg.linear.x = MOVE_VEL; 
            this->y++;
            break; 
        case DOWN: 
            this->twist_msg.linear.x = -MOVE_VEL;
            this->y--;
            break; 
        default: 
            break;  
    }
 
    // do it
    this->twist_pub.publish(this->twist_msg); 
  
    // wait until we see a circle again
/*    while (this->circles.empty());
    while (!this->circles.empty());
    while (this->circles.empty());*/
    //while (this->img_vec.empty()); 
    //while (!this->img_vec.empty()); 
    //while (this->img_vec.empty()); 
    // stop-gap time-based motion
    ros::Duration(8.2).sleep();

    // stop
    this->twist_msg.linear.x = 0; 
    this->twist_msg.linear.y = 0; 
    this->twist_msg.linear.z = 0; 
    this->twist_pub.publish(this->twist_msg);
    ros::Duration(2).sleep(); 
}
