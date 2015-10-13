#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <human_catching/HumanFall.h>

namespace {
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
public:
	SimulatedFallDetector() :
		pnh("~") {
         fallPub = nh.advertise<human_catching::HumanFall>(
				"out", 1);

        ros::service::waitForService("/gazebo/get_model_state");
        modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persistent */);
        
        publishFall(getHumanPosition());
	}

private:
    geometry_msgs::Point getHumanPosition(){
        gazebo_msgs::GetModelState modelState;
        modelState.request.model_name = "human";
        modelStateServ.call(modelState);
        return modelState.response.pose.position;
    }
    
    void publishFall(const geometry_msgs::Point& torsoLocation){
        human_catching::HumanFall fall;
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
	ros::init(argc, argv, "simulated_fall_detector");

	SimulatedFallDetector sfd;
	ros::spin();
}
