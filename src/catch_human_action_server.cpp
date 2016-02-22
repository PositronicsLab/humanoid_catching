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

static const double EPSILON = 0.1;
static const double PLANNING_TIME = 0.25;
static const string ARMS[] = {"left_arm", "right_arm"};
static const string ARM_TOPICS[] = {"l_arm_force_controller/command", "r_arm_force_controller/command"};

static const ros::Duration SEARCH_RESOLUTION(0.1);
static const double pi = boost::math::constants::pi<double>();

struct Solution {
    unsigned int armsSolved;
    geometry_msgs::Pose pose;
    map<string, vector<double> > jointPositions;
    ros::Duration goalTime;
    ros::Duration delta;
};

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
        }

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
        ROS_INFO("Waiting for transform");

        // Transform to the base_frame for IK, which is torso_lift_link
        if(!tf.waitForTransform(predictFall.response.header.frame_id, "/torso_lift_link", predictFall.response.header.stamp, ros::Duration(15))){
            ROS_WARN("Failed to get transform");
            as.setAborted();
            return;
        }

        // Transform the goal to the base frame, as it becomes the obstacle
        if(!tf.waitForTransform(goal->header.frame_id, "/torso_lift_link", goal->header.stamp, ros::Duration(15))){
            ROS_WARN("Failed to get transform");
            as.setAborted();
            return;
        }

        ROS_INFO("Transforms aquired");

        // We now have a projected time/position path. Search the path for acceptable times.
        vector<Solution> solutions;

        // Epsilon causes us to always select a position in front of the fall.
        ros::Duration lastTime;
        unsigned int ops = 0;
        double timePerOp = 0;

        for (vector<FallPoint>::const_iterator i = predictFall.response.points.begin(); i != predictFall.response.points.end(); ++i) {

            // Determine if we should search this point.
            if (lastTime != ros::Duration(0) && i->time - lastTime < SEARCH_RESOLUTION) {
                continue;
            }

            lastTime = i->time;

            Solution possibleSolution;
            possibleSolution.delta = ros::Duration(1000); // Initialize max delta to large number
            possibleSolution.armsSolved = 0;
            possibleSolution.pose = i->pose;
            possibleSolution.goalTime = ros::Duration(i->time);

            geometry_msgs::PoseStamped basePose;
            basePose.header = predictFall.response.header;
            basePose.pose = i->pose;

            geometry_msgs::PoseStamped transformedPose;
            transformedPose.header.frame_id = "/torso_lift_link";
            transformedPose.header.stamp = predictFall.response.header.stamp;
            try {
                tf.transformPose(transformedPose.header.frame_id, transformedPose.header.stamp, basePose,
                                     predictFall.response.header.frame_id, transformedPose);
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("Failed to transform pose from %s to %s due to %s", predictFall.response.header.frame_id.c_str(),
                        "/torso_lift_link", ex.what());
                continue;
            }

            // Lookup the IK solution
            kinematics_cache::IKQuery ikQuery;
            ikQuery.request.pose = transformedPose;
            // TODO: Determine appropriate value
            ikQuery.request.error = 0.025;

            boost::timer opTimer;
            ops++;
            if (!ik.call(ikQuery)) {
                ROS_DEBUG("Failed to find IK solution for arms");
                timePerOp += opTimer.elapsed() / float(ops);
                break;
            } else {
                ROS_DEBUG("Received %lu results from IK query", ikQuery.response.results.size());
            }
            timePerOp += opTimer.elapsed() / float(ops);

            // Now check if the time is feasible
            // TODO: All times are currently reporting as infeasible.
            /*
            for (IKList::iterator j = ikQuery.response.results.begin(); j != ikQuery.response.results.end(); ++j) {
                if (ros::Duration((1 + EPSILON) * j->simulated_execution_time.toSec()) + ros::Duration(PLANNING_TIME) > i->time) {
                    ROS_INFO("Position could not be reached in time. Execution time is %f, epsilon is %f, and fall time is %f",
                            j->simulated_execution_time.toSec(), EPSILON, i->time.toSec());
                            break;
                }
            }
            */

            bool armsSolved[2] = {false, false};
            for (IKList::iterator j = ikQuery.response.results.begin(); j != ikQuery.response.results.end(); ++j) {
                possibleSolution.delta = min(possibleSolution.delta, ros::Duration(i->time) - j->simulated_execution_time);
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

        ROS_INFO("Average time per query was %f(s). n = %u", timePerOp, ops);

        if (solutions.empty()) {
            ROS_WARN("No possible catch positions");
            as.setAborted();
            return;
        }

        ROS_INFO("Selecting optimal position from %lu poses", solutions.size());

        // Select the point which is reachable most quicly
        ros::Duration highestDeltaTime = ros::Duration(-10);
        unsigned int highestArmsSolved = 0;

        vector<Solution>::iterator bestSolution = solutions.end();
        for (vector<Solution>::iterator solution = solutions.begin(); solution != solutions.end(); ++solution) {

            // Always prefer higher armed solutions
            if (solution->armsSolved > highestArmsSolved || solution->armsSolved == highestArmsSolved && solution->delta > highestDeltaTime) {
                highestArmsSolved = solution->armsSolved;
                highestDeltaTime = solution->delta;
                ROS_INFO("Selected a pose with more arms solved or a higher delta time. Pose is %f %f %f, arms solved is %u, and delta is %f", solution->pose.position.x,
                         solution->pose.position.y, solution->pose.position.z, highestArmsSolved, highestDeltaTime.toSec());
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
        geometry_msgs::PoseStamped obstacle;
        obstacle.header.frame_id = "/torso_lift_link";
        obstacle.header.stamp = goal->header.stamp;

        geometry_msgs::PoseStamped obstacleStamped;
        obstacleStamped.header = goal->header;
        obstacleStamped.pose = goal->pose;
        try {
            tf.transformPose(obstacle.header.frame_id, obstacle.header.stamp, obstacleStamped,
                             goal->header.frame_id, obstacle);
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR("Failed to transform pose from %s to %s due to %s", obstacleStamped.header.frame_id.c_str(),
                      "/torso_lift_link", ex.what());
            return;
        }

        // Now move both arms to the position
        for (unsigned int i = 0; i < armCommandPubs.size(); ++i) {
            if (bestSolution->jointPositions.find(ARMS[i]) != bestSolution->jointPositions.end()) {
                humanoid_catching::Move command;
                command.header.stamp = goal->header.stamp;
                command.header.frame_id = "torso_lift_link";
                command.target = bestSolution->pose;
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
