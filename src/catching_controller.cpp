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
#include <boost/timer.hpp>
#include <boost/math/constants/constants.hpp>

namespace {
using namespace std;
using namespace human_catching;

static const double EPSILON = 0.1;
static const double PLANNING_TIME = 0.25;
static const string ARMS[] = {"left", "right"};
static const ros::Duration SEARCH_RESOLUTION(0.1);
static const double pi = boost::math::constants::pi<double>();

typedef actionlib::SimpleActionClient<human_catching::MoveArmFastAction> ArmClient;

enum ARM {
    LEFT = 0,
    RIGHT = 1,
};

struct Solution {
    unsigned int armsSolved;
    geometry_msgs::Pose pose;
    vector<vector<double> > jointPositions;
    ros::Duration goalTime;
    ros::Duration delta;
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
        ROS_INFO("Beginning procedure to catch human");
        ros::WallTime startWallTime = ros::WallTime::now();
        ros::Time startRosTime = ros::Time::now();

        human_catching::PredictFall predictFall;
        predictFall.request.header = imuData->header;
        predictFall.request.pose = imuData->pose;
        predictFall.request.twist = imuData->twist;

        ROS_INFO("Predicting fall");
        if (!fallPredictor.call(predictFall)) {
            ROS_WARN("Fall prediction failed");
            return;
        }

        ROS_INFO("Fall predicted successfully");
        ROS_INFO("Waiting for transform");

        // Transform to the base_frame for IK, which is torso_lift_link
        if(!tf.waitForTransform(predictFall.response.header.frame_id, "/torso_lift_link", predictFall.response.header.stamp, ros::Duration(15))){
            ROS_WARN("Failed to get transform");
            return;
        }

        ROS_INFO("Transform aquired");

        // We now have a projected time/position path. Search the path for acceptable times.
        vector<Solution> solutions;

        // Epsilon causes us to always select a position in front of the fall.
        ros::Duration lastTime;
        unsigned int ops = 0;
        double timePerOp = 0;

        for (unsigned int i = 0; i < predictFall.response.times.size(); ++i) {

            // Determine if we should search this point.
            if (i != 0 && predictFall.response.times[i] - lastTime < SEARCH_RESOLUTION) {
                continue;
            }

            lastTime = predictFall.response.times[i];

            Solution possibleSolution;
            possibleSolution.delta = ros::Duration(1000); // Initialize max delta to large number
            possibleSolution.armsSolved = 0;
            possibleSolution.pose = predictFall.response.path[i];
            possibleSolution.goalTime = ros::Duration(predictFall.response.times[i]);

            geometry_msgs::PoseStamped basePose;
            basePose.header = predictFall.response.header;
            basePose.pose = predictFall.response.path[i];

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

            for (unsigned int j = 0; j < boost::size(arms); ++j) {
                // Lookup the IK solution
                kinematics_cache::IKQuery ikQuery;
                ikQuery.request.group = ARMS[j] + "_arm";
                ikQuery.request.pose = transformedPose;

                // Offset so end effectors are not in same space. This is in torso lift link coordinates, which is a plane
                // parallel to the cartesian plane.
                if (j == LEFT) {
                    ikQuery.request.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-pi / 2.0, 0, pi / 2.0);
                    ikQuery.request.pose.pose.position.z += 0.05;
                } else {
                    ikQuery.request.pose.pose.position.z -= 0.05;
                    ikQuery.request.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pi / 2.0, 0, -pi / 2.0);
                }

                boost::timer opTimer;
                ops++;
                if (!ik.call(ikQuery)) {
                    ROS_DEBUG("Failed to find IK solution for arm %s", ARMS[j].c_str());
                    timePerOp += opTimer.elapsed() / float(ops);
                    break;
                }
                timePerOp += opTimer.elapsed() / float(ops);

                if (ikQuery.response.positions.size() == 0) {
                    ROS_ERROR("Response from IK cache was invalid");
                    break;
                }

                // Now check if the time is feasible
                if ((1 + EPSILON) * ikQuery.response.execution_time.toSec() + PLANNING_TIME > predictFall.response.times[i]) {
                    ROS_INFO("Position could not be reached in time. Execution time is %f, epsilon is %f, and fall time is %f",
                             ikQuery.response.execution_time.toSec(), EPSILON, predictFall.response.times[i]);
                             // TODO: All times are currently reporting as infeasible.
                             // break;
                }

                ROS_INFO("Position could be reached in time. Execution time is %f, epsilon is %f, and fall time is %f",
                         ikQuery.response.execution_time.toSec(), EPSILON, predictFall.response.times[i].toSec());

                possibleSolution.armsSolved++;
                possibleSolution.delta = min(possibleSolution.delta, ros::Duration(predictFall.response.times[i]) - ikQuery.response.execution_time);
                possibleSolution.jointPositions.push_back(ikQuery.response.positions);
            }
            solutions.push_back(possibleSolution);
        }

        ROS_INFO("Average time per query was %f(s). n = %u", timePerOp, ops);

        if (solutions.empty()) {
            ROS_WARN("No possible catch positions");
            return;
        }

        ROS_INFO("Selecting optimal position from %lu poses", solutions.size());

        // Select the point which is reachable most quicly
        ros::Duration highestDeltaTime = ros::Duration(-10);
        vector<Solution>::iterator bestSolution;
        for (vector<Solution>::iterator solution = solutions.begin(); solution != solutions.end(); ++solution) {
            if (solution->armsSolved != 2) {
                ROS_INFO("Skipping solution without 2 arms solved");
                continue;
            }

            if (solution->delta > highestDeltaTime) {
                highestDeltaTime = solution->delta;
                ROS_DEBUG("Selected a pose with a higher delta time. Pose is %f %f %f and delta is %f", solution->pose.position.x,
                         solution->pose.position.y, solution->pose.position.z, highestDeltaTime.toSec());
                bestSolution = solution;
            }
        }

        ROS_INFO("Reaction time was %f(s) wall time and %f(s) clock time", ros::WallTime::now().toSec() - startWallTime.toSec(),
                 ros::Time::now().toSec() - startRosTime.toSec());

        // Now move both arms to the position
        for (unsigned int i = 0; i < arms.size(); ++i) {
            human_catching::MoveArmFastGoal goal;
            goal.joint_positions = bestSolution->jointPositions[i];
            goal.goal_time = bestSolution->goalTime;
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
