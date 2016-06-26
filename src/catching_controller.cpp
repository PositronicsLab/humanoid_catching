#include <ros/ros.h>
#include <humanoid_catching/HumanFall.h>
#include <humanoid_catching/CatchHumanAction.h>
#include <actionlib/client/simple_action_client.h>
#include <message_filters/subscriber.h>
#include <humanoid_catching/IMU.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

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
    auto_ptr<message_filters::Subscriber<HumanFall> > humanFallSub;

    //! Human IMU subscriber
    auto_ptr<message_filters::Subscriber<humanoid_catching::IMU> > humanIMUSub;

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
                new message_filters::Subscriber<humanoid_catching::IMU>(nh, "/human/imu", 1));
        humanIMUSub->unsubscribe();

        ROS_INFO("Waiting for catch human server");
        catchHumanClient.reset(new CatchHumanClient("catch_human_action", true));
        catchHumanClient->waitForServer();

        imuVizPub = nh.advertise<geometry_msgs::PoseStamped>("/catching_controller/imu_data", 1);

        ROS_INFO("Catching controller initialized successfully");
	}

private:
    void fallDetected(const HumanFallConstPtr& fallingMsg) {
        ROS_INFO("Human fall detected at @ %f", fallingMsg->header.stamp.toSec());

        // Unsubscribe from further fall notifications
        humanFallSub->unsubscribe();

        // Begin listening for IMU notifications
        humanIMUSub->subscribe();
        humanIMUSub->registerCallback(boost::bind(&CatchingController::imuDataDetected, this, _1));

        // Wait to receive a IMU notification to take action so we are
        // aware of any initial velocity
    }

    void visualizeImuMsg(const humanoid_catching::IMUConstPtr data) const {
        if (imuVizPub.getNumSubscribers() > 0) {
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header = data->header;
            poseStamped.pose = data->pose;
            imuVizPub.publish(poseStamped);
        }
    }

    void imuDataDetected(const humanoid_catching::IMUConstPtr& imuData) {
        ROS_INFO("Human IMU data received at @ %f", imuData->header.stamp.toSec());

        // Publish the viz message
        visualizeImuMsg(imuData);
        catchHuman(imuData);
    }

    void catchHuman(const humanoid_catching::IMUConstPtr& imuData) {
        ROS_INFO("Beginning procedure to catch human");
        humanoid_catching::CatchHumanGoal goal;
        goal.header = imuData->header;
        goal.velocity = imuData->velocity;
        goal.accel = imuData->acceleration;
        goal.pose = imuData->pose;
        actionlib::SimpleClientGoalState gs = catchHumanClient->sendGoalAndWait(goal, ros::Duration(MAX_CATCH_TIME), ros::Duration(MAX_PREEMPT_TIME));
        if (gs.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Human was initially caught successfully");
        } else {
            ROS_INFO("Human was not caught successfully. Failure state was %s", gs.getText().c_str());
        }
    }
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "catching_controller");

	CatchingController cc;
	ros::spin();
}
