#include <ros/ros.h>
#include <human_catching/HumanFall.h>
#include <message_filters/subscriber.h>
#include <human_catching/IMU.h>
#include <human_catching/PredictFall.h>
#include <kinematics_cache/IKQuery.h>
#include <boost/range.hpp>
#include <actionlib/client/simple_action_client.h>
#include <human_catching/MoveArmFastAction.h>
#include <memory>

namespace {
using namespace std;
using namespace human_catching;

static const double EPSILON = 0.1;
static const string ARMS[] = {"left", "right"};

typedef actionlib::SimpleActionClient<human_catching::MoveArmFastAction> ArmClient;

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

    //! Cached fall prediction client
    ros::ServiceClient fallPredictor;

    //! Cached IK client.
    ros::ServiceClient ik;

    //! Arm clients
    vector<boost::shared_ptr<ArmClient> > arms;

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

        // Configure the fall prediction service
        ros::service::waitForService("/fall_predictor/predict_fall");
        fallPredictor = nh.serviceClient<human_catching::PredictFall>("/fall_predictor/predict_fall", true /* persistent */);

        ros::service::waitForService("/kinematics_cache/ik");
        ik = nh.serviceClient<kinematics_cache::IKQuery>("/kinematics_cache/ik", true /* persistent */);

        // Initialize arm clients
        for (unsigned int i = 0; i < boost::size(ARMS); ++i) {
            ArmClient* arm = new ArmClient(ARMS[i] + "_move_arm_fast_action_server", true);
            arms.push_back(boost::shared_ptr<ArmClient>(arm));
            arms[i]->waitForServer();
        }

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

        catchHuman(imuData);
    }

    // TODO: Move this to async action
    void catchHuman(const human_catching::IMUConstPtr& imuData) {
        human_catching::PredictFall predictFall;
        predictFall.request.header = imuData->header;
        predictFall.request.pose = imuData->pose;
        predictFall.request.twist = imuData->twist;

        if (!fallPredictor.call(predictFall)) {
            ROS_WARN("Fall prediction failed");
            return;
        }

        // TODO: Need to redo the kinematic cache time predictions.
        // We now have a projected time/position path. Search the path for acceptable times.

        vector<geometry_msgs::Pose> possiblePoses;

        // TODO: Set the pose to the one used to load the IK cache

        // TODO: Adjust offsets per arm
        for (unsigned int i = 0; i < predictFall.response.times.size(); ++i) {
            for (unsigned int j = 0; boost::size(arms); ++j) {
                // Lookup the IK solution
                kinematics_cache::IKQuery ikQuery;
                ikQuery.request.group = ARMS[j] + "_arm";
                ikQuery.request.pose.header = predictFall.response.header;
                ikQuery.request.pose.pose = predictFall.response.path[i];
                if (!ik.call(ikQuery)) {
                    ROS_INFO("Failed to find IK solution for arm %s", ARMS[j].c_str());
                    continue;
                }

                // Now check if the time is feasible
                if ((1 + EPSILON) * ikQuery.response.execution_time.toSec() > predictFall.response.times[i]) {
                    ROS_INFO("Position could not be reached in time. Execution time is %f, epsilon is %f, and fall time is %f",
                             ikQuery.response.execution_time.toSec(), EPSILON, predictFall.response.times[i]);
                }

                // Check if this both arms meet criteria
                if (j == 1) {
                    ROS_INFO("Found acceptable position");
                    possiblePoses.push_back(predictFall.response.path[i]);
                }
            }
        }

        if (possiblePoses.empty()) {
            ROS_WARN("No possible catch positions");
            return;
        }

        // Select the highest point as this maximizes possible energy dissipation
        double highestZ = 0;
        geometry_msgs::Pose bestPose;
        for (unsigned int i = 0; i < possiblePoses.size(); ++i) {
            if (possiblePoses[i].position.z > highestZ) {
                highestZ = possiblePoses[i].position.z;
                bestPose = possiblePoses[i];
            }
        }

        // Now move both arms to the position
        for (unsigned int i = 0; i < arms.size(); ++i) {
            human_catching::MoveArmFastGoal goal;
            goal.target.header = predictFall.response.header;
            goal.target.pose = bestPose;
            arms[i]->sendGoal(goal);
        }

        // TODO: What error handling should we do here? Any preemption?
    }
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "catching_controller");

	CatchingController cc;
	ros::spin();
}
