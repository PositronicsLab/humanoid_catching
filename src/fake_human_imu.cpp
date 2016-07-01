#include <ros/ros.h>
#include <humanoid_catching/IMU.h>
#include <gazebo_msgs/GetModelState.h>
#include <humanoid_catching/HumanFall.h>
#include <message_filters/subscriber.h>

namespace {
using namespace std;
using namespace geometry_msgs;
using namespace humanoid_catching;

static const double FREQUENCY = 0.001;

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

    //! Human fall subscriber
    auto_ptr<message_filters::Subscriber<HumanFall> > humanFallSub;

public:
	FakeHumanIMU() :
		pnh("~") {
         humanPosePub = nh.advertise<humanoid_catching::IMU>(
				"out", 1);

        ros::service::waitForService("/gazebo/get_model_state");
        modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persistent */);

        humanFallSub.reset(new message_filters::Subscriber<HumanFall>(nh, "/human/fall", 1));
        humanFallSub->registerCallback(boost::bind(&FakeHumanIMU::fallDetected, this, _1));
	}

private:

    void fallDetected(const HumanFallConstPtr& fallingMsg) {
        ROS_DEBUG("Human fall detected at @ %f", fallingMsg->header.stamp.toSec());
        humanFallSub->unsubscribe();
        timer = nh.createTimer(ros::Duration(FREQUENCY), &FakeHumanIMU::callback, this);
        timer.start();
    }

    humanoid_catching::IMU getIMUData(){
        gazebo_msgs::GetModelState modelState;
        modelState.request.model_name = "human";
        modelStateServ.call(modelState);
        humanoid_catching::IMU data;
        data.header.stamp = ros::Time::now();
        data.header.frame_id = "/odom_combined";

        data.velocity = modelState.response.twist;
        data.pose = modelState.response.pose;

        ROS_INFO("True position: %f %f %f", data.pose.position.x, data.pose.position.y, data.pose.position.z);

        // Clear fields that are not available from IMU
        data.pose.position.x = data.pose.position.y = data.pose.position.z = 0;
        return data;
    }

    void callback(const ros::TimerEvent& event){

        // Lookup the current IMU data for the human
        humanoid_catching::IMU data = getIMUData();

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
