#include <ros/ros.h>
#include <human_catching/IMU.h>
#include <gazebo_msgs/GetModelState.h>

namespace {
using namespace std;
using namespace geometry_msgs;

static const double FREQUENCY = 0.01;

class FakeHumanIMU {
private:
    //! Publisher for the human pose
	ros::Publisher humanPosePub;

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

    //! Frequency at which to publish the humanoid state
    ros::Timer timer;

    //! Cached service client.
    ros::ServiceClient modelStateServ;
public:
	FakeHumanIMU() :
		pnh("~") {
         humanPosePub = nh.advertise<human_catching::IMU>(
				"out", 1);

        ros::service::waitForService("/gazebo/get_model_state");
        modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persistent */);
        timer = nh.createTimer(ros::Duration(FREQUENCY), &FakeHumanIMU::callback, this);
        timer.start();
	}

private:
    human_catching::IMU getIMUData(){
        gazebo_msgs::GetModelState modelState;
        modelState.request.model_name = "human";
        modelStateServ.call(modelState);
        human_catching::IMU data;
        data.header.stamp = ros::Time::now();
        data.header.frame_id = "/map";
        data.twist = modelState.response.twist;
        data.pose = modelState.response.pose;
        return data;
    }

    void callback(const ros::TimerEvent& event){

        // Lookup the current IMU data for the human
        human_catching::IMU data = getIMUData();

        // Publish the event
        ROS_DEBUG_STREAM("Publishing a human IMU event: " << data);
        humanPosePub.publish(data);
      }
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "fake_human_imu");

	FakeHumanIMU fhi;
	ros::spin();
}
