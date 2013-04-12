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
#include <cstdlib>

// ardrone_autonomy
#include "ardrone_autonomy/CamSelect.h"
#include "ardrone_autonomy/Navdata.h"

// ardrone_thinc
#include "ArdroneThinc.hpp"

using namespace cv;
using namespace std;

namespace smsg = std_msgs;
namespace ssrv = std_srvs;
using geometry_msgs::Twist;
using sensor_msgs::Image;
using ardrone_autonomy::Navdata;
using ardrone_autonomy::CamSelect;

int main(int argc, char *argv[]) {
    // ros initialization
    ros::init(argc, argv, "thinc_main");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // data container
    ArdroneThinc at; 
   
    // handle usage
    if (argc != 3) {
        cout << "usage: ";
        cout << argv[0];
        cout << " <cols> <rows> <drone-id> <drone-col> <drone-row>";
        cout << endl;
        exit(1); 
    }

    // grid size
    at.columns = atoi(argv[1]);
    at.rows = atoi(argv[2]);

    // initial position and id
    at.id = atoi(argv[3]);
    at.x = atoi(argv[4]);
    at.y = atoi(argv[5]);
    
    // publishers
    at.launch_pub = n.advertise<smsg::Empty>("ardrone/takeoff", 5);
    at.land_pub = n.advertise<smsg::Empty>("ardrone/land", 5);
    at.reset_pub = n.advertise<smsg::Empty>("ardrone/reset", 5);
    at.twist_pub = n.advertise<Twist>("cmd_vel", 10);
    at.thresh_pub = n.advertise<Image>("img_thresh", 10);

    // subscribers
    at.cam_sub = n.subscribe<Image>("ardrone/bottom/image_raw", 1,
            &ArdroneThinc::CamCallback, &at);
    at.nav_sub = n.subscribe<Navdata>("ardrone/navdata", 1,
            &ArdroneThinc::NavdataCallback, &at);

    // service clients
    at.camchan_cli = n.serviceClient<CamSelect>("ardrone/setcamchannel", 1);
    at.trim_cli = n.serviceClient<ssrv::Empty>("ardrone/flattrim");

    // services
    at.waypoint_srv = n.advertiseService("waypoint",
            &ArdroneThinc::WaypointCallback, &at);

    // let roscore catch up
    ros::Duration(1.0).sleep();

    // set camchannel on drone and takeoff
    if(ros::ok()) {
        // set camera to bottom
        ardrone_autonomy::CamSelect camchan_req;
        camchan_req.request.channel = 1;
        at.camchan_cli.call(camchan_req);

        // calibrate to flat surface
        ssrv::Empty trim_req;
        at.trim_cli.call(trim_req);

        // hover initially and launch
        at.twist_msg.linear.x = 0;
        at.twist_msg.linear.y = 0;
        at.twist_msg.linear.z = 0;
        at.launch_pub.publish(at.empty_msg);
    } 
    
    ros::spin();

    return 0;
}
