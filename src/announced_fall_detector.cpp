#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <humanoid_catching/HumanFall.h>

namespace {
using namespace std;
using namespace geometry_msgs;

// TODO: Calibrate this height
static const double DEFAULT_HEIGHT = 0.5;

class AnnouncedFallDetector {
private:
    //! Publisher for the fall information
	ros::Publisher fallPub;

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;
public:
	AnnouncedFallDetector() :
		pnh("~") {
         fallPub = nh.advertise<humanoid_catching::HumanFall>(
				"out", 1);
         geometry_msgs::Point torsoLocation;
         pnh.param("x", torsoLocation.x, 0.0);
         pnh.param("y", torsoLocation.y, 0.0);
         pnh.param("z", torsoLocation.z, DEFAULT_HEIGHT);
         publishFall(torsoLocation);
	}

private:
    void publishFall(const geometry_msgs::Point& torsoLocation){
        humanoid_catching::HumanFall fall;
        fall.position = torsoLocation;
        fall.header.frame_id = "/map";
        fall.header.stamp = ros::Time::now();

        // Publish the event
        ROS_DEBUG("Publishing a human fall event");
        fallPub.publish(fall);
      }
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "announced_fall_detector");

	AnnouncedFallDetector ahf;
	ros::spin();
}
