#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <humanoid_catching/HumanFall.h>

namespace {

static const double EPSILON = 0.075;
static const double TIMER_FREQ = 0.01;

using namespace std;

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
         fallPub = nh.advertise<humanoid_catching::HumanFall>(
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
            publishFall();
        }
      }

      void publishFall() {
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
	ros::init(argc, argv, "simulated_fall_detector");

	SimulatedFallDetector sfd;
	ros::spin();
}
