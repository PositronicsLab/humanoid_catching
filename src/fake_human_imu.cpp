#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/GetModelState.h>

namespace {
using namespace std;
using namespace geometry_msgs;

class FakeHumanIMU {
private:
    //! Publisher for the human pose
	ros::Publisher humanPosePub;
    
	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;
    
    //! Frequency at which to publish the humanoid twist
    ros::Timer timer;
    
    //! Cached service client.
    ros::ServiceClient modelStateServ;
public:
	FakeHumanIMU() :
		pnh("~") {
         humanPosePub = nh.advertise<geometry_msgs::TwistStamped>(
				"out", 1);
                
        ros::service::waitForService("/gazebo/get_model_state");
        modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persistent */);
        timer = nh.createTimer(ros::Duration(0.1), &FakeHumanIMU::callback, this);
        timer.start();
	}
    
private:
    geometry_msgs::TwistStamped getHumanTwist(){
        gazebo_msgs::GetModelState modelState;
        modelState.request.model_name = "human";
        modelStateServ.call(modelState);
        geometry_msgs::TwistStamped twist;
        twist.header.stamp = ros::Time::now();
        twist.header.frame_id = "/map";
        twist.twist = modelState.response.twist;
        return twist;
    }
        
    void callback(const ros::TimerEvent& event){
        
        // Lookup the current twist of the human
        geometry_msgs::TwistStamped twist = getHumanTwist();

        // Publish the event
        ROS_DEBUG("Publishing a human twist event");
        humanPosePub.publish(twist);
      }
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "fake_human_imu");

	FakeHumanIMU fhi;
	ros::spin();
}
