#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <humanoid_catching/CatchHumanAction.h>
#include <actionlib/client/simple_action_client.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

namespace {
using namespace std;
using namespace humanoid_catching;

static const double MAX_CATCH_TIME = 5.0;
static const double MAX_PREEMPT_TIME = 0.5;

typedef actionlib::SimpleActionClient<humanoid_catching::CatchHumanAction> CatchHumanClient;

class CatchingController {
private:

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

    //! Visualization of imu data
    ros::Publisher imuVizPub;

    //! Human fall subscriber
    auto_ptr<message_filters::Subscriber<std_msgs::Header> > humanFallSub;

    //! Reset sub
    auto_ptr<message_filters::Subscriber<std_msgs::Header> > resetSub;

    //! Human IMU subscriber
    auto_ptr<message_filters::Subscriber<sensor_msgs::Imu> > humanIMUSub;

    //! Catch human action client.
    auto_ptr<CatchHumanClient> catchHumanClient;

public:
	CatchingController() :
		pnh("~") {
        ROS_INFO("Initializing the catching controller");
        humanFallSub.reset(
                new message_filters::Subscriber<std_msgs::Header>(nh, "/human/fall", 1));
        humanFallSub->registerCallback(boost::bind(&CatchingController::fallDetected, this, _1));

        resetSub.reset(
                new message_filters::Subscriber<std_msgs::Header>(nh, "/catching_controller/reset", 1));
        resetSub->registerCallback(boost::bind(&CatchingController::reset, this, _1));

        // Construct but don't initialize
        humanIMUSub.reset(
                new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/in", 1));
        humanIMUSub->unsubscribe();

        ROS_DEBUG("Waiting for catch human server");
        catchHumanClient.reset(new CatchHumanClient("catch_human_action", true));
        catchHumanClient->waitForServer();

        ROS_INFO("Catching controller initialized successfully");
	}

private:
    void fallDetected(const std_msgs::HeaderConstPtr& fallingMsg) {
        ROS_INFO("Human fall detected at @ %f", fallingMsg->stamp.toSec());

        // Unsubscribe from further fall notifications
        humanFallSub->unsubscribe();

        // Begin listening for IMU notifications
        humanIMUSub->subscribe();
        humanIMUSub->registerCallback(boost::bind(&CatchingController::imuDataDetected, this, _1));

        // Wait to receive a IMU notification to take action so we are
        // aware of any initial velocity
    }

    void imuDataDetected(const sensor_msgs::ImuConstPtr& imuData) {
        ROS_DEBUG("Human IMU data received at @ %f", imuData->header.stamp.toSec());
        catchHuman(imuData);
    }

    void catchHuman(const sensor_msgs::ImuConstPtr& imuData) {
        ROS_DEBUG("Beginning procedure to catch human");
        humanoid_catching::CatchHumanGoal goal;
        goal.header = imuData->header;
        goal.velocity.angular = imuData->angular_velocity;
        goal.accel.linear = imuData->linear_acceleration;
        goal.orientation = imuData->orientation;
        actionlib::SimpleClientGoalState gs = catchHumanClient->sendGoalAndWait(goal, ros::Duration(MAX_CATCH_TIME), ros::Duration(MAX_PREEMPT_TIME));
        if (gs.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_DEBUG("Human was initially caught successfully");
        } else {
            ROS_DEBUG("Human was not caught successfully. Failure state was %s", gs.getText().c_str());
        }
    }

    void reset(const std_msgs::HeaderConstPtr& reset) {
        ROS_INFO("Resetting catching controller");

        // Stop the current actions
        humanIMUSub->unsubscribe();

        // Begin listening for IMU notifications
        humanFallSub->subscribe();
    }
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "catching_controller");

	CatchingController cc;
	ros::spin();
}
