// ros
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"

// opencv2
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

// ardrone_autonomy
#include "ardrone_autonomy/CamSelect.h"

// ardrone_thinc
#include "ArdroneThinc.hpp"

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    // ros init
    ros::init(argc, argv, "thinc_main");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // publisher, subscribers, and serviceclients
    ArdroneThinc at;
    at.launch = n.advertise<std_msgs::Empty>("ardrone/takeoff", 5);
    at.land = n.advertise<std_msgs::Empty>("ardrone/land", 5);
    at.reset = n.advertise<std_msgs::Empty>("ardrone/reset", 5);
    at.twist = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    at.thresh = n.advertise<sensor_msgs::Image>("thinc/thresh", 10);
    at.cam = n.subscribe<sensor_msgs::Image>("ardrone/image_raw", 1,
            &ArdroneThinc::CamCallback, &at);
    at.camchannel = n.serviceClient<ardrone_autonomy::CamSelect>("ardrone/setcamchannel");
    at.flattrim = n.serviceClient<std_srvs::Empty>("ardrone/flattrim");

    // sleep to allow everything to register with roscore
    ros::Duration(1.0).sleep();

    // set camchannel on drone and takeoff
    if(ros::ok()) {
        // set camera to bottom
        ardrone_autonomy::CamSelect camsrv;
        camsrv.request.channel = 1;
        at.camchannel.call(camsrv);

        // calibrate to flat surface
        std_srvs::Empty trimsrv;
        at.flattrim.call(trimsrv);

        // hover initially
        at.twist_msg.linear.x = 0;
        at.twist_msg.linear.y = 0;
        at.twist_msg.linear.z = 0;
        at.launch.publish(at.empty_msg);
    } 
    
    // hover drone in place
    while(ros::ok()) {
        ros::spinOnce(); // see ArdroneThinc.CamCallback()
        loop_rate.sleep();
    }

    return 0;
}
