#include <ros/ros.h>
#include <humanoid_catching/HumanFall.h>

namespace {
using namespace std;

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
         publishFall();
	}

private:
    void publishFall(){
        humanoid_catching::HumanFall fall;
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
