#include <ros/ros.h>
#include <human_catching/HumanFall.h>
#include <human_catching/CatchHumanAction.h>
#include <actionlib/client/simple_action_client.h>
#include <message_filters/subscriber.h>
#include <human_catching/IMU.h>

namespace {
using namespace std;
using namespace human_catching;

typedef actionlib::SimpleActionClient<human_catching::CatchHumanAction> CatchHumanClient;

class CatchingController {
private:

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

    //! Human fall subscriber
    auto_ptr<message_filters::Subscriber<HumanFall> > humanFallSub;

    //! Human IMU subscriber
    auto_ptr<message_filters::Subscriber<human_catching::IMU> > humanIMUSub;

    //! Catch human action client.
    auto_ptr<CatchHumanClient> catchHumanClient;
public:
	CatchingController() :
		pnh("~") {
        ROS_INFO("Initializing the catching controller");
        humanFallSub.reset(
                new message_filters::Subscriber<HumanFall>(nh, "/human/fall", 1));
        humanFallSub->registerCallback(boost::bind(&CatchingController::fallDetected, this, _1));

        // Construct but don't initialize
        humanIMUSub.reset(
                new message_filters::Subscriber<human_catching::IMU>(nh, "/human/imu", 1));

        ROS_INFO("Waiting for catch human server");
        catchHumanClient.reset(new CatchHumanClient("catch_human_action", true));
        catchHumanClient->waitForServer();

        ROS_INFO("Catching controller initialized successfully");
	}

private:
    void fallDetected(const HumanFallConstPtr& fallingMsg) {
        ROS_INFO("Human fall detected at @ %f", fallingMsg->header.stamp.toSec());

        // Unsubscribe from further fall notifications
        humanFallSub->unsubscribe();

        // Begin listening for IMU notifications
        humanIMUSub->registerCallback(boost::bind(&CatchingController::imuDataDetected, this, _1));

        // Wait to receive a IMU notification to take action so we are
        // aware of any initial velocity
    }

    void imuDataDetected(const human_catching::IMUConstPtr& imuData) {
        ROS_INFO("Human IMU data detected at @ %f", imuData->header.stamp.toSec());

        // Stop listening for IMU data
        humanIMUSub->unsubscribe();

        catchHuman(imuData);
    }

    void catchHuman(const human_catching::IMUConstPtr& imuData) {
        ROS_INFO("Beginning procedure to catch human");
        human_catching::CatchHumanGoal goal;
        goal.header = imuData->header;
        goal.twist = imuData->twist;
        goal.pose = imuData->pose;
        catchHumanClient->sendGoal(goal);
    }
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "catching_controller");

	CatchingController cc;
	ros::spin();
}
