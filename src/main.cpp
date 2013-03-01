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

// C++
#include <vector>

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

    ArdroneThinc at;
    
    //first two arguments are columns and rows, respectively.
    int c, r;
    stringstream s1(argv[0]);
    s1 >> c;
    stringstream s2(argv[1]);
    s2 >> r;

    //after columns and rows, arguments proceed as follows: 
    //drone name, spawn x position, spawn y position, and
    //repeat n times for n drones
    vector<drone*> drones;
    ros::init(argc, argv, "mover");
    for (int i = 0; i < (argc-2)/3; i++) {
        int x, y, id;
        string id_string = argv[2 + 3*i];
        string x_string = argv[3 + 3*i];
        string y_string = argv[4 + 3*i];
        stringstream s3(id_string); 
        s3 >> id; 
        stringstream s4(x_string);
        s4 >> x;
        stringstream s5(y_string);
        s5 >> y;

        drone* d = new drone(id, x, y);
        drones.push_back(d);

        //advertise
        at.launch_publishers[i] = n.advertise<std_msgs::Empty>("drone" + id_string + "ardrone/takeoff", 5); 
        at.land_publishers[i] = n.advertise<std_msgs::Empty>("drone" + id_string + "ardrone/land", 5);
        at.reset_publishers[i] = n.advertise<std_msgs::Empty>("drone" + id_string + "ardrone/reset", 5);
        at.twist_publishers[i] = n.advertise<geometry_msgs::Twist>("drone" + id_stirng + "cmd_vel", 10);
        at.thresh_publishers[i] = n.advertise<sensor_msgs::Image>("drone" + id_string + "thinc/thresh", 10);
        at.cam_subscribers[i] = n.subscribe<sensor_msgs::Image>("drone" + id_string + "ardrone/image_raw", 1,
            &ArdroneThinc::CamCallback, &at);
        at.camchannel_clients[i] = n.serviceClient<ardrone_autonomy::CamSelect>("drone" id_string + "ardrone/setcamchannel");
        at.flattrim_clients[i] = n.serviceClient<std_srvs::Empty>("drone" + id_string + "ardrone/flattrim");

    }


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
        //at.launch.publish(at.empty_msg);
    } 
    
    // hover drone in place
    while(ros::ok()) {
        ros::spinOnce(); // see ArdroneThinc.CamCallback()
        loop_rate.sleep();
    }

    return 0;
}
