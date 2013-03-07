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
    int new_h = orig->image.size.p[0];
    int new_w = (new_h*orig->image.size.p[1])/orig->image.size.p[0];
    resize(orig->image, orig->image, Size(new_w, new_h), 0, 0);

    // detect circles using hough transform
    cvtColor(orig->image, grey->image, CV_RGB2GRAY); // rgb -> grey
    GaussianBlur(grey->image, grey->image, Size(3, 3), 0); // denoise
    vector<Vec3f> c;
    HoughCircles(grey->image, c, CV_HOUGH_GRADIENT, 2, 5, 220, 120);
    for(size_t i = 0; i < c.size(); i++) {
        Point center(cvRound(c[i][0]), cvRound(c[i][1]));
        int radius = cvRound(c[i][2]);
        circle(orig->image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        circle(orig->image, center, radius, Scalar(255, 0, 0), 3, 8, 0);
    }

    // convert opencv image to ros image and publish
    //thresh.publish(orig->toImageMsg()); //giving error?
    
    // center drone
    //if(xp > UB) twist_msg.linear.y = -VEL;
    //else if(xp < LB) twist_msg.linear.y = VEL;
    //else if(LB < xp && xp < UB) twist_msg.linear.y = 0;
    //if(yp > UB) twist_msg.linear.x = -VEL;
    //else if(yp < LB) twist_msg.linear.x = VEL;
    //else if(LB < yp && yp < UB) twist_msg.linear.x = 0;
    //twist.publish(twist_msg);
}

/*
 * Callback function of the Waypoint_Navigator Service.
 * Determines how much to move and calls move()
 * accordingly. Returns true if successfuly, otherwise
 * false.
 */
bool ArdroneThinc::Waypoint_Navigator_Callback(ardrone_thinc::Waypoint_Navigator::Request &req, ardrone_thinc::Waypoint_Navigator::Response &res) {
    
    drone* d = drones[req.id]; 
    
    if (is_valid_grid_cell(req.x, req.y)) {
        if (d != NULL) {
            bool move_right = false;
            bool move_left = false;
            bool move_up = false;
            bool move_down = false;

            int delta_x = d->grid_pos[0] - req.x;
            int delta_y = d->grid_pos[1] - req.y;

            if (delta_x < 0)
                move_right = true;
            else if (delta_x > 0)
                move_left = true;
            if (delta_y < 0)
                move_up = true;
            else if (delta_y > 0)
                move_down = true;

            int x_moves = abs(delta_x);
            int y_moves = abs(delta_y);

            while (ros::ok() && (x_moves > 0 || y_moves > 0)) {
                //complete all moves in x-direction first
                if (x_moves > 0) {
                    if (move_right) 
                        move(req.id, 'r'); 
                    else if (move_left) 
                        move(req.id, 'l'); 
                    x_moves--; 
                }

                //complete all moves in y-direction second
                else if (y_moves > 0) {
                    if (move_up) 
                        move(req.id, 'u');               
                    else if (move_down) 
                        move(req.id, 'd'); 
                    y_moves--; 
                } 
            }    
        }
    }

    return false;
}

/*
 * Move the drone with the given id one cell in the 
 * direction indicated- 'r' = right, 'l' = left, 
 * 'u' = up, and 'd' = down.
 */
bool ArdroneThinc::move(int id, char direction) {

    /* -linear.x: move backward
     * +linear.x: move forward
     * -linear.y: move right
     * +linear.y: move left
     */

    switch(direction) {
        case 'l': 
            twist_msg.linear.y = 0.25; 
            break; 
        case 'r': 
            twist_msg.linear.y = -0.25; 
            break; 
        case 'u': 
            twist_msg.linear.x = 0.25; 
            break; 
        case 'd': 
            twist_msg.linear.x = -0.25; 
            break; 
        default: 
            return false; 
    }

    twist_publishers[id].publish(twist_msg); 
    ros::Duration(1000.0).sleep(); 

    twist_msg.linear.x = 0; //reset values of twist after we move
    twist_msg.linear.y = 0; 
    twist_msg.linear.z = 0; 
    return true; 
}

/*
 * Determine if the given grid cell (x,y) is valid.
 * Return true if it is, otherwise return false.
 */
bool ArdroneThinc::is_valid_grid_cell(int x, int y) {
    if (x < 0 || y < 0 || x > columns || y > rows)
        return false;

    return true;
}



/*
 * Create function to add drones to vector from thinc_main.
 * Give access to number of columns and rows in grid.
 */
