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
#include <tf/transform_listener.h>

namespace {
using namespace std;
using namespace human_catching;

static const double EPSILON = 0.1;
static const double PLANNING_TIME = 0.25;
static const string ARMS[] = {"left", "right"};
static const double SEARCH_RESOLUTION = 0.1;

typedef actionlib::SimpleActionClient<human_catching::MoveArmFastAction> ArmClient;

enum ARM {
    LEFT = 0,
    RIGHT = 1,
};

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

    //! TF listener
    tf::TransformListener tf;
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
        ROS_INFO("Waiting for predict_fall service");
        ros::service::waitForService("/fall_predictor/predict_fall");
        fallPredictor = nh.serviceClient<human_catching::PredictFall>("/fall_predictor/predict_fall", true /* persistent */);

        ROS_INFO("Waiting for kinematics_cache/ik service");
        ros::service::waitForService("/kinematics_cache/ik");
        ik = nh.serviceClient<kinematics_cache::IKQuery>("/kinematics_cache/ik", true /* persistent */);

        // Initialize arm clients
        ROS_INFO("Initializing move_arm_fast_action_servers");
        for (unsigned int i = 0; i < boost::size(ARMS); ++i) {
            ArmClient* arm = new ArmClient(ARMS[i] + "_arm_move_arm_fast_action_server", true);
            arms.push_back(boost::shared_ptr<ArmClient>(arm));
            ROS_INFO("Waiting for %s", (ARMS[i] + "_arm_move_arm_fast_action_server").c_str());
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

        // Stop listening for IMU data
        humanIMUSub->unsubscribe();
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

        // Transform to the base_frame for IK, which is torso_lift_link
        if(!tf.waitForTransform(predictFall.response.header.frame_id, "/torso_lift_link", predictFall.response.header.stamp, ros::Duration(5))){
            ROS_WARN("Failed to get transform");
            return;
        }

        // TODO: Need to redo the kinematic cache time predictions.
        // We now have a projected time/position path. Search the path for acceptable times.

        vector<geometry_msgs::Pose> possiblePoses;

        // TODO: Use a better data structure
        vector<vector<vector<double> > > jointSolutions;

        // TODO: Set the pose to the one used to load the IK cache

        // Epsilon causes us to always select a position in front of the fall.
        // TODO: Orient hands towards each other?
        double lastTime = 0;
        for (unsigned int i = 0; i < predictFall.response.times.size(); ++i) {

            // Determine if we should search this point.
            if (i != 0 && predictFall.response.times[i] - lastTime < SEARCH_RESOLUTION) {
                continue;
            }
            lastTime = predictFall.response.times[i];

            vector<double> firstArmSolution;
            for (unsigned int j = 0; j < boost::size(arms); ++j) {
                // Lookup the IK solution
                kinematics_cache::IKQuery ikQuery;
                ikQuery.request.group = ARMS[j] + "_arm";

                geometry_msgs::PoseStamped basePose;
                basePose.header = predictFall.response.header;
                basePose.pose = predictFall.response.path[i];

                // Offset so end effectors are not in same space. This is in cartesian coordinates.
                if (j == LEFT) {
                    basePose.pose.position.z += 0.05;
                } else {
                    basePose.pose.position.z -= 0.05;
                }

                geometry_msgs::PoseStamped transformedPose;
                transformedPose.header.frame_id = "/torso_lift_link";
                transformedPose.header.stamp = predictFall.response.header.stamp;
                try {
                    tf.transformPose(transformedPose.header.frame_id, transformedPose.header.stamp, basePose,
                                     predictFall.response.header.frame_id, transformedPose);
                }
                catch (tf::TransformException& ex) {
                    ROS_ERROR("Failed to transform pose from %s to %s", predictFall.response.header.frame_id.c_str(),
                        "/torso_lift_link");
                    continue;
                }

                ikQuery.request.pose = transformedPose;
                if (!ik.call(ikQuery)) {
                    ROS_DEBUG("Failed to find IK solution for arm %s", ARMS[j].c_str());
                    continue;
                }

                // Now check if the time is feasible
                if ((1 + EPSILON) * ikQuery.response.execution_time.toSec() + PLANNING_TIME > predictFall.response.times[i]) {
                    ROS_INFO("Position could not be reached in time. Execution time is %f, epsilon is %f, and fall time is %f",
                             ikQuery.response.execution_time.toSec(), EPSILON, predictFall.response.times[i]);
                } else {
                    ROS_INFO("Position could be reached in time. Execution time is %f, epsilon is %f, and fall time is %f",
                             ikQuery.response.execution_time.toSec(), EPSILON, predictFall.response.times[i]);
                }

                if (j == 0) {
                    firstArmSolution = ikQuery.response.positions;
                }

                // Check if this both arms meet criteria
                if (j == 1) {
                    ROS_INFO("Found acceptable position");
                    possiblePoses.push_back(predictFall.response.path[i]);

                    // Save the solution
                    vector<vector<double> > armSolutions(2);
                    armSolutions[0] = firstArmSolution;
                    armSolutions[1] = ikQuery.response.positions;
                    jointSolutions.push_back(armSolutions);
                }
            }
        }

        if (possiblePoses.empty()) {
            ROS_WARN("No possible catch positions");
            return;
        }

        ROS_INFO("Selecting optimal position from %lu poses", possiblePoses.size());
        assert(possiblePoses.size() == jointSolutions.size());

        // Select the highest point as this maximizes possible energy dissipation
        // TODO: Could also select maximum time delta
        double highestZ = 0;
        geometry_msgs::Pose bestPose;
        vector<vector<double> > bestJointPositions;
        for (unsigned int i = 0; i < possiblePoses.size(); ++i) {
            if (possiblePoses[i].position.z > highestZ) {
                ROS_INFO("Selected a pose with a higher Z value. Pose is %f %f %f", possiblePoses[i].position.x, possiblePoses[i].position.y, possiblePoses[i].position.z);
                highestZ = possiblePoses[i].position.z;
                bestPose = possiblePoses[i];
                bestJointPositions = jointSolutions[i];
            }
        }

        // Now move both arms to the position
        for (unsigned int i = 0; i < arms.size(); ++i) {
            human_catching::MoveArmFastGoal goal;
            goal.joint_positions = bestJointPositions[i];
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
