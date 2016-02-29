#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <humanoid_catching/CatchHumanAction.h>
#include <kinematics_cache/IKQuery.h>
#include <humanoid_catching/PredictFall.h>
#include <actionlib/client/simple_action_client.h>
#include <humanoid_catching/Move.h>
#include <tf/transform_listener.h>
#include <boost/timer.hpp>
#include <boost/math/constants/constants.hpp>
#include <map>

namespace {
using namespace std;
using namespace humanoid_catching;

typedef actionlib::SimpleActionServer<humanoid_catching::CatchHumanAction> Server;
typedef vector<kinematics_cache::IK> IKList;

static const string ARMS[] = {"left_arm", "right_arm"};
static const string ARM_TOPICS[] = {"l_arm_force_controller/command", "r_arm_force_controller/command"};
static const string ARM_GOAL_VIZ_TOPICS[] = {"/catch_human_action_server/movement_goal/left_arm", "/catch_human_action_server/movement_goal/right_arm"};

static const ros::Duration SEARCH_RESOLUTION(0.10);
static const double pi = boost::math::constants::pi<double>();

struct Solution {
    unsigned int armsSolved;
    geometry_msgs::PoseStamped pose;
    map<string, vector<double> > jointPositions;
    ros::Duration goalTime;
    ros::Duration delta;
};

static geometry_msgs::Pose applyTransform(const geometry_msgs::PoseStamped& pose, const tf::StampedTransform transform){
    tf::Stamped<tf::Pose> tfPose;
    tf::poseStampedMsgToTF(pose, tfPose);
    tf::Pose tfGoal = transform * tfPose;
    geometry_msgs::Pose msgGoal;
    tf::poseTFToMsg(tfGoal, msgGoal);
    return msgGoal;
}

class CatchHumanActionServer {
private:

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

    //! Action Server
    Server as;

    //! Cached fall prediction client
    ros::ServiceClient fallPredictor;

    //! Cached IK client.
    ros::ServiceClient ik;

    //! Arm clients
    vector<ros::Publisher> armCommandPubs;

    //! TF listener
    tf::TransformListener tf;

    //! Visualization of goals
    vector<ros::Publisher> goalPubs;

    //! Visualization of trials
    ros::Publisher trialGoalPub;

    //! Create messages that are used to published feedback/result
    humanoid_catching::CatchHumanFeedback feedback;
    humanoid_catching::CatchHumanResult result;
public:
	CatchHumanActionServer(const string& name) :
		pnh("~"),
       as(nh, name, boost::bind(&CatchHumanActionServer::execute, this, _1), false) {
        ROS_INFO("Initializing the catch human action");

        // Configure the fall prediction service
        ROS_INFO("Waiting for predict_fall service");
        ros::service::waitForService("/fall_predictor/predict_fall");
        fallPredictor = nh.serviceClient<humanoid_catching::PredictFall>("/fall_predictor/predict_fall", true /* persistent */);

        ROS_INFO("Waiting for kinematics_cache/ik service");
        ros::service::waitForService("/kinematics_cache/ik");
        ik = nh.serviceClient<kinematics_cache::IKQuery>("/kinematics_cache/ik", true /* persistent */);

        // Initialize arm clients
        ROS_INFO("Initializing arm command publishers");
        for (unsigned int i = 0; i < boost::size(ARMS); ++i) {
            armCommandPubs.push_back(nh.advertise<humanoid_catching::Move>(ARM_TOPICS[i], 1));
            goalPubs.push_back(nh.advertise<geometry_msgs::PointStamped>(ARM_GOAL_VIZ_TOPICS[i], 1));
        }

        trialGoalPub = nh.advertise<geometry_msgs::PointStamped>("/catch_human_action_server/movement_goal_trials", 1);

        ROS_INFO("Starting the action server");
        as.registerPreemptCallback(boost::bind(&CatchHumanActionServer::preempt, this));
        as.start();
        ROS_INFO("Catch human action initialized successfully");
	}

private:
    void preempt() {
        // Currently no way to cancel force controllers.
        as.setPreempted();
    }

    void visualizeGoal(const geometry_msgs::PoseStamped& goal, unsigned int armIndex) const {
        if (goalPubs[armIndex].getNumSubscribers() > 0) {
            geometry_msgs::PointStamped point;
            point.header = goal.header;
            point.point = goal.pose.position;
            goalPubs[armIndex].publish(point);
        }
    }

    static geometry_msgs::PointStamped poseToPoint(const geometry_msgs::PoseStamped pose) {
        geometry_msgs::PointStamped point;
        point.point = pose.pose.position;
        point.header = pose.header;
        return point;
    }

    void execute(const humanoid_catching::CatchHumanGoalConstPtr& goal){

        ROS_INFO("Catching human");

        if(!as.isActive() || as.isPreemptRequested() || !ros::ok()){
            ROS_INFO("Catch human action cancelled before started");
            return;
        }

        ros::WallTime startWallTime = ros::WallTime::now();
        ros::Time startRosTime = ros::Time::now();

        humanoid_catching::PredictFall predictFall;
        predictFall.request.header = goal->header;
        predictFall.request.pose = goal->pose;
        predictFall.request.velocity = goal->velocity;
        predictFall.request.accel = goal->accel;

        ROS_INFO("Predicting fall");
        if (!fallPredictor.call(predictFall)) {
            ROS_WARN("Fall prediction failed");
            as.setAborted();
            return;
        }
        ROS_INFO("Fall predicted successfully");

        // Transform to the base_frame of the human (and therefore the obstacle) for IK, which is torso_lift_link
        ROS_INFO("Waiting for transform");
        if(!tf.waitForTransform(predictFall.response.header.frame_id, "/torso_lift_link", predictFall.response.header.stamp, ros::Duration(15))){
            ROS_WARN("Failed to get transform");
            as.setAborted();
            return;
        }

        tf::StampedTransform goalToTorsoTransform;
        tf.lookupTransform("/torso_lift_link", predictFall.response.header.frame_id, predictFall.response.header.stamp, goalToTorsoTransform);

        ROS_INFO("Transforms aquired");

        // We now have a projected time/position path. Search the path for acceptable times.
        vector<Solution> solutions;

        // Epsilon causes us to always select a position in front of the fall.
        ros::Duration lastTime;

        for (vector<FallPoint>::const_iterator i = predictFall.response.points.begin(); i != predictFall.response.points.end(); ++i) {

            // Determine if we should search this point.
            if (lastTime != ros::Duration(0) && i->time - lastTime < SEARCH_RESOLUTION) {
                continue;
            }

            lastTime = i->time;

            Solution possibleSolution;
            // Initialize to a large negative number.
            possibleSolution.delta = ros::Duration(-1000);
            possibleSolution.armsSolved = 0;
            possibleSolution.goalTime = ros::Duration(i->time);

            geometry_msgs::PoseStamped basePose;
            basePose.header = predictFall.response.header;
            basePose.pose = i->pose;

            geometry_msgs::PoseStamped transformedPose;
            transformedPose.header.frame_id = "/torso_lift_link";
            transformedPose.header.stamp = predictFall.response.header.stamp;

            transformedPose.pose = applyTransform(basePose, goalToTorsoTransform);
            possibleSolution.pose = transformedPose;

            if (trialGoalPub.getNumSubscribers() > 0) {
                trialGoalPub.publish(poseToPoint(transformedPose));
            }

            // Lookup the IK solution
            kinematics_cache::IKQuery ikQuery;
            ikQuery.request.pose = transformedPose;

            if (!ik.call(ikQuery)) {
                ROS_DEBUG("Failed to find IK solution for arms");
                continue;
            }

            ROS_DEBUG("Received %lu results from IK query", ikQuery.response.results.size());

            bool armsSolved[2] = {false, false};
            for (IKList::iterator j = ikQuery.response.results.begin(); j != ikQuery.response.results.end(); ++j) {
                // We want to search for the fastest arm movement. The idea is that the first arm there should slow the human
                // down and give the second arm time to arrive.
                possibleSolution.delta = max(possibleSolution.delta, ros::Duration(i->time) - j->simulated_execution_time);
                possibleSolution.jointPositions[j->group] = j->positions;

                for (unsigned int k = 0; k < boost::size(ARMS); ++k) {
                    if (j->group == ARMS[k]) {
                        armsSolved[k] = true;
                    }
                }
             }

             for (unsigned int k = 0; k < boost::size(ARMS); ++k) {
                if (armsSolved[k]) {
                    possibleSolution.armsSolved++;
                }
             }
             solutions.push_back(possibleSolution);
        }

        if (solutions.empty()) {
            ROS_WARN("No possible catch positions");
            as.setAborted();
            return;
        }

        ROS_INFO("Selecting optimal position from %lu poses", solutions.size());

        // Select the point which is reachable most quicly
        ros::Duration highestDeltaTime = ros::Duration(-1000);
        unsigned int highestArmsSolved = 0;

        vector<Solution>::iterator bestSolution = solutions.end();
        for (vector<Solution>::iterator solution = solutions.begin(); solution != solutions.end(); ++solution) {

            // Always prefer higher armed solutions
            if (solution->armsSolved > highestArmsSolved || solution->armsSolved == highestArmsSolved && solution->delta > highestDeltaTime) {
                highestArmsSolved = solution->armsSolved;
                highestDeltaTime = solution->delta;
                ROS_INFO("Selected a pose with more arms solved or a higher delta time. Pose is %f %f %f, arms solved is %u, and delta is %f", solution->pose.pose.position.x,
                         solution->pose.pose.position.y, solution->pose.pose.position.z, highestArmsSolved, highestDeltaTime.toSec());
                bestSolution = solution;
            }
        }

        ROS_INFO("Reaction time was %f(s) wall time and %f(s) clock time", ros::WallTime::now().toSec() - startWallTime.toSec(),
                 ros::Time::now().toSec() - startRosTime.toSec());

        if (bestSolution == solutions.end()) {
            ROS_WARN("No solutions found");
            as.setAborted();
            return;
        }

        // Convert the obstacle to the movement frame
        geometry_msgs::PoseStamped obstacleStamped;
        obstacleStamped.header = goal->header;
        obstacleStamped.pose = goal->pose;

        geometry_msgs::PoseStamped obstacle;
        obstacle.pose = applyTransform(obstacleStamped, goalToTorsoTransform);
        obstacle.header.frame_id = "/torso_lift_link";
        obstacle.header.stamp = goal->header.stamp;



        // Now move both arms to the position
        for (unsigned int i = 0; i < armCommandPubs.size(); ++i) {
            if (bestSolution->jointPositions.find(ARMS[i]) != bestSolution->jointPositions.end()) {
                humanoid_catching::Move command;
                visualizeGoal(bestSolution->pose, i);
                command.header = bestSolution->pose.header;
                command.target = bestSolution->pose.pose;
                command.obstacle = obstacle.pose;
                command.has_obstacle = true;
                armCommandPubs[i].publish(command);
            } else {
                ROS_INFO("Skipping arm %s due to no solution.", ARMS[i].c_str());
            }
        }
        as.setSucceeded();
    }
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "catch_human_action");

	CatchHumanActionServer cha(ros::this_node::getName());
	ros::spin();
}
