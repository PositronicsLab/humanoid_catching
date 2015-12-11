#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <human_catching/HumanFall.h>

namespace {

static const double EPSILON = 0.1;
static const double TIMER_FREQ = 0.01;

using namespace std;
using namespace geometry_msgs;

class SimulatedFallDetector {
private:
    //! Publisher for the fall information
	ros::Publisher fallPub;

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

    //! Cached service client.
    ros::ServiceClient modelStateServ;

    //! Frequency at which to query the humanoid state
    ros::Timer timer;
public:
	SimulatedFallDetector() :
		pnh("~") {
         fallPub = nh.advertise<human_catching::HumanFall>(
				"out", 1);

        ros::service::waitForService("/gazebo/get_model_state");
        modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persistent */);
        timer = nh.createTimer(ros::Duration(TIMER_FREQ), &SimulatedFallDetector::callback, this);
        timer.start();
	}

private:

    void callback(const ros::TimerEvent& event){
        gazebo_msgs::GetModelState modelState;
        modelState.request.model_name = "human";
        modelStateServ.call(modelState);

        // Check for a z velocity
        if (modelState.response.twist.linear.z < -EPSILON || fabs(modelState.response.twist.linear.x) > EPSILON || fabs(modelState.response.twist.linear.y) > EPSILON) {
            ROS_DEBUG("Detected negative Z velocity");
            publishFall(modelState.response.pose.position);
        }
      }

      void publishFall(const geometry_msgs::Point torsoLocation) {
        human_catching::HumanFall fall;
        fall.position = torsoLocation;
        fall.header.frame_id = "/map";
        fall.header.stamp = ros::Time::now();

        // Publish the event
        ROS_INFO("Publishing a human fall event");
        fallPub.publish(fall);
      }
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "simulated_fall_detector");

	SimulatedFallDetector sfd;
	ros::spin();
}
