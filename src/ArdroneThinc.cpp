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
#include <fstream>
#include <cmath>
#include <iostream>

// degree to radian helper macro
#define D2R(a) (a*M_PI/180)

using namespace cv;

using std::cout;
using std::endl;
using sensor_msgs::ImageConstPtr;
using ardrone_autonomy::NavdataConstPtr;
using ardrone_thinc::Waypoint;
using ardrone_thinc::PrintNavdata;

// MOVE_VEL is for use in the simulator
#define MOVE_VEL 1

// REAL_MOVE_VEL is for real world drones
#define REAL_MOVE_VEL 0.1

// threshold images and adjust drone position accordingly
void ArdroneThinc::CamCallback(const sensor_msgs::ImageConstPtr& rosimg) {
/*	cout << "Entered cam callback method" << endl;
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

    img_vec = c;
    Point avg_center; // grab from laptop code
    for(size_t i = 0; i < c.size(); i++) {
        Point center(cvRound(c[i][0]), cvRound(c[i][1]));
        int radius = cvRound(c[i][2]);
        avg_center += (Point(c[i][0], c[i][1]) - avg_center)*(1.0/(i+1));
        circle(orig->image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        circle(orig->image, center, radius, Scalar(0, 0, 255), 3, 8, 0);
    }

    double tan2x = tan(D2R(this->rotx))*tan(D2R(this->rotx));
    double tan2y = tan(D2R(this->roty))*tan(D2R(this->roty));
    double height = this->sonar/sqrt(1+tan2x+tan2y);
    Point over(height*sin(D2R(rotx)), height*sin(D2R(roty)));

    const double WFOV = 58;
    const double HFOV = 47;
    double w_angle = WFOV/2*(2*avg_center.x/new_w - 1);
    double h_angle = HFOV/2*(2*avg_center.y/new_h - 1);
    Point avg_center_mm(height*tan(w_angle), height*tan(h_angle));
    Point move = over - avg_center_mm;

    // convert opencv image to ros image and publish
    //this->thresh_pub.publish(orig->toImageMsg());
    

    double xp = move.x;
    double yp = move.y;
    
    // center drone
    if(xp > this->y) {
	cout<< "if 1" << endl;
	twist_msg.linear.y = -REAL_MOVE_VEL;
    }
    else if(xp < this->y) {
	cout<< "if 2" << endl;
	twist_msg.linear.y = REAL_MOVE_VEL;
    }
    else if(this->y < xp && xp < this->y) {
	cout<< "if 3" << endl;
	twist_msg.linear.y = 0;
    }
    if(yp > this->x) {
	cout<< "if 4" << endl;
	twist_msg.linear.x = -REAL_MOVE_VEL;
    }
    else if(yp < this->x) {
	cout<< "if 5" << endl;
	twist_msg.linear.x = REAL_MOVE_VEL;
    }
    else if(this->x < yp && yp < this->x) {
	cout<< "if 6" << endl;
	twist_msg.linear.x = 0;
    }
    twist_pub.publish(twist_msg);    */
}

// collect navdata
void ArdroneThinc::NavdataCallback(const NavdataConstPtr& nav) {
    this->rotx = nav->rotX;
    this->roty = nav->rotY;
    this->sonar = nav->altd;
    this->batteryPercent = nav->batteryPercent;
    this->vx = nav->vx;
    this->vy = nav->vy;
    this->vz = nav->vz;
}

//print navdata to client, used in gatac
bool ArdroneThinc::PrintNavdataCallback(PrintNavdata::Request &req, PrintNavdata::Response &res) {
	int opt = req.option;
	cout<< "Navdata print request, option "<< opt << endl;

    float batteryConvert = this->batteryPercent;
    stringstream ss0 (stringstream::in | stringstream::out);
    ss0 << batteryConvert;
    string batteryString = ss0.str();
    this->batteryCurrent= "Battery percent: " + batteryString;

    float forVelocConvert = this->vy;
    stringstream ss1 (stringstream::in | stringstream::out);
    ss1 << forVelocConvert;
    string forVelocString = ss1.str();
    this->forwardVelocityCurrent  = "Forward velocity: " + forVelocString;

    float sideVelocConvert = this->vx;
    stringstream ss2 (stringstream::in | stringstream::out);
    ss2 << sideVelocConvert;
    string sideVelocString = ss2.str();
    this->sidewaysVelocityCurrent = "Sideways velocity: " + sideVelocString;

    float vertVelocConvert = this->vz;
    stringstream ss3 (stringstream::in | stringstream::out);
    ss3 << vertVelocConvert;
    string vertVelocString = ss3.str();
    this->verticalVelocityCurrent  = "Vertical velocity: " + vertVelocString;

    int sonarConvert = this->sonar;
    stringstream ss4 (stringstream::in | stringstream::out);
    ss4 << sonarConvert;
    string sonarString = ss4.str();
    this->sonarCurrent  = "Sonar reading: " + sonarString;


    ofstream file ("currentNavdata.txt");
    if (file.is_open())
    {
    file << this->batteryCurrent <<"\n";
    file << this->forwardVelocityCurrent <<"\n";
    file << this->sidewaysVelocityCurrent <<"\n";
    file << this->verticalVelocityCurrent <<"\n";
    file << this->sonarCurrent <<"\n";
 
    }
return true;
}

// move to designated sector
bool ArdroneThinc::WaypointCallback(Waypoint::Request &req, Waypoint::Response &res) {
    cout << "waypoint request: ";
    cout << req.x << ", " << req.y << endl;

    // ensure valid grid cell
    if(req.x < 0 || req.y < 0 || req.x >= this->columns || req.y >= this->rows) {
        return false; 
    }

    // calculate how many cells to move
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

    // -linear.x: move backward
    // +linear.x: move forward
    // -linear.y: move right
    // +linear.y: move left
    if(simDrones == false){
    switch(d) {
        case LEFT: 
            this->twist_msg.linear.y = REAL_MOVE_VEL; 
            this->x--;
            break; 
        case RIGHT: 
            this->twist_msg.linear.y = -REAL_MOVE_VEL;
            this->x++;
            break; 
        case UP: 
            this->twist_msg.linear.x = REAL_MOVE_VEL; 
            this->y++;
            break; 
        case DOWN: 
            this->twist_msg.linear.x = -REAL_MOVE_VEL;
            this->y--;
            break; 
        default: 
            break;  
    }
    }
    else if(simDrones == true){
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
    }
    // publish message to move
    this->twist_pub.publish(this->twist_msg); 
  
    // stop-gap time-based motion, for simulator
    if(simDrones == true)
    ros::Duration(2.1).sleep();

    // stop-gap time-based motion, for real drones
    if(simDrones == false)
    ros::Duration(1.5).sleep();

    // stop moving and hover
    this->twist_msg.linear.x = 0; 
    this->twist_msg.linear.y = 0; 
    this->twist_msg.linear.z = 0; 
    this->twist_msg.angular.x = 0; 
    this->twist_msg.angular.y = 0; 
    this->twist_msg.angular.z = 0; 
    this->twist_pub.publish(this->twist_msg);
    ros::Duration(2).sleep(); 
}
