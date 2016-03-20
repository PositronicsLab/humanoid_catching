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
#include <moveit/rdf_loader/rdf_loader.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <message_filters/subscriber.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace {
using namespace std;
using namespace humanoid_catching;

typedef actionlib::SimpleActionServer<humanoid_catching::CatchHumanAction> Server;
typedef vector<kinematics_cache::IK> IKList;

static const string ARMS[] = {"left_arm", "right_arm"};
static const string ARM_TOPICS[] = {"l_arm_force_controller/command", "r_arm_force_controller/command"};
static const string ARM_GOAL_VIZ_TOPICS[] = {"/catch_human_action_server/movement_goal/left_arm", "/catch_human_action_server/movement_goal/right_arm"};
static const double MAX_VELOCITY = 100;
static const double MAX_ACCELERATION = 100;

static const ros::Duration SEARCH_RESOLUTION(0.10);
static const double pi = boost::math::constants::pi<double>();

struct Limit {
    Limit() : velocity(0.0), acceleration(0.0) {}
    Limit(double aVelocity, double aAcceleration) : velocity(aVelocity), acceleration(aAcceleration) {}
    double velocity;
    double acceleration;
};
typedef std::map<std::string, Limit> LimitMapType;

struct State {
    State() : position(0.0), velocity(0.0) {}
    State(double aPosition, double aVelocity) : position(aPosition), velocity(aVelocity) {}
    double position;
    double velocity;
};

typedef std::map<std::string, State> StateMapType;

struct Solution {
    bool armsSolved[2];
    bool feasable;
    geometry_msgs::PoseStamped pose;
    ros::Duration goalTime;
    ros::Duration delta;

    Solution(){
        armsSolved[0] = armsSolved[1] = false;
        feasable = false;
    }
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

    //! Subscriber for joint state updates
    auto_ptr<message_filters::Subscriber<sensor_msgs::JointState> > jointStatesSub;

    //! Arm clients
    vector<ros::Publisher> armCommandPubs;

    //! TF listener
    tf::TransformListener tf;

    //! Visualization of goals
    vector<ros::Publisher> goalPubs;

    //! Visualization of trials
    ros::Publisher trialGoalPub;

    //! Current joint states
    StateMapType jointStates;

    //! Joint limits
    LimitMapType jointLimits;

    //! Joint names
    map<string, vector<string> > jointNames;

    //! Planning scene
    boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planningScene;

    //! Create messages that are used to published feedback/result
    humanoid_catching::CatchHumanFeedback feedback;
    humanoid_catching::CatchHumanResult result;
private:
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        ROS_DEBUG("Received a joint states message");
        for(unsigned int i = 0; i < msg->name.size(); ++i) {
            StateMapType::iterator iter = jointStates.find(msg->name[i]);
            if (iter != jointStates.end()) {
                iter->second.position = msg->position[i];
                iter->second.velocity = msg->velocity[i];
            }
        }
    }

public:
	CatchHumanActionServer(const string& name) :
		pnh("~"),
        as(nh, name, boost::bind(&CatchHumanActionServer::execute, this, _1), false),
        planningScene(new planning_scene_monitor::PlanningSceneMonitor("robot_description")) {

        ROS_INFO("Initializing the catch human action");

        // Configure the planning scene
        planningScene->startStateMonitor();
        planningScene->startSceneMonitor();
        planningScene->startWorldGeometryMonitor();

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

        rdf_loader::RDFLoader rdfLoader;
        const boost::shared_ptr<srdf::Model> &srdf = rdfLoader.getSRDF();
        const boost::shared_ptr<urdf::ModelInterface>& urdfModel = rdfLoader.getURDF();
        robot_model::RobotModelPtr kinematicModel(new robot_model::RobotModel(urdfModel, srdf));
        ROS_INFO("Robot model initialized successfully");

        for (unsigned int k = 0; k < boost::size(ARMS); ++k) {
            const robot_model::JointModelGroup* jointModelGroup =  kinematicModel->getJointModelGroup(ARMS[k]);

            ROS_INFO("Loading joint limits for arm %s", ARMS[k].c_str());
            for (unsigned int i = 0; i < jointModelGroup->getActiveJointModels().size(); ++i) {
                const string& jointName = jointModelGroup->getActiveJointModels()[i]->getName();
                jointNames[ARMS[k]].push_back(jointName);

                // Set default position and velocity
                jointStates[jointName] = State(0.0, 0.0);

                const string prefix = "robot_description_planning/joint_limits/" + jointName + "/";
                ROS_INFO_NAMED("catch_human_action_server", "Loading velocity and accleration limits for joint %s", jointName.c_str());

                bool has_vel_limits;
                double max_velocity;
                if (nh.getParam(prefix + "has_velocity_limits", has_vel_limits) && has_vel_limits && nh.getParam(prefix + "max_velocity", max_velocity)) {
                    ROS_INFO_NAMED("catch_human_action_server", "Setting max velocity to %f", max_velocity);
                } else {
                    ROS_INFO_NAMED("catch_human_action_server", "Setting max velocity to default");
                    max_velocity = MAX_VELOCITY;
                }

                bool has_acc_limits;
                double max_acc;
                if (nh.getParam(prefix + "has_acceleration_limits", has_acc_limits) && has_acc_limits && nh.getParam(prefix + "max_acceleration", max_acc)) {
                    ROS_INFO_NAMED("catch_human_action_server", "Setting max acceleration to %f", max_acc);
                } else {
                    ROS_INFO_NAMED("catch_human_action_server", "Setting max acceleration to default");
                    max_acc = MAX_ACCELERATION;
                }

                jointLimits[jointName] = Limit(max_velocity, max_acc);
            }
        }

        ROS_INFO("Completed initializing the joint limits");

        jointStatesSub.reset(new message_filters::Subscriber<sensor_msgs::JointState>(nh, "/joint_states", 5));
        jointStatesSub->registerCallback(boost::bind(&CatchHumanActionServer::jointStatesCallback, this, _1));

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

    // TODO: This shares a lot of code with the moveit_plugin
    // TODO: Incorporate current velocity
    ros::Duration calcExecutionTime(const string& group, const vector<double>& solution) {

        double longestTime = 0.0;
        for(unsigned int i = 0; i < solution.size(); ++i) {
            const string& jointName = jointNames[group][i];
            Limit& limits = jointLimits[jointName];

            ROS_DEBUG_NAMED("catch_human_action_server", "Calculating distance for joint %u", i);
            double d = fabs(jointStates[jointName].position - solution[i]);
            ROS_DEBUG_NAMED("catch_human_action_server", "Distance to travel is %f", d);

            // Determine the max "bang-bang" distance.
            double x = 2.0 * limits.velocity / limits.acceleration;
            double max_tri_distance = 0.5 * limits.velocity * x;
            ROS_DEBUG_NAMED("catch_human_action_server", "Maximum triangular distance given max_vel %f and max_accel %f is %f", limits.velocity, limits.acceleration, max_tri_distance);

            double t;
            if (d <= max_tri_distance) {
                t = sqrt(d / limits.acceleration);
                ROS_DEBUG_NAMED("catch_human_action_server", "Triangular solution for t is %f", t);
            }
            else {
                // Remove acceleration and deacceleration distance and calculate the trapezoidal base distance
                double d_rect = d - max_tri_distance;
                double x_rect = d_rect / limits.velocity;
                t = sqrt(max_tri_distance / limits.acceleration) + x_rect;
                ROS_DEBUG_NAMED("ikfast", "Trapezoidal solution for t is %f", t);
            }

            longestTime = max(longestTime, t);
        }

        ROS_DEBUG_NAMED("catch_human_action_server", "Execution time is %f", longestTime);
        return ros::Duration(longestTime);
    }

    static unsigned int numArmsSolved(const Solution& solution) {
        unsigned int numArmsSolved = 0;
        for (unsigned int k = 0; k < boost::size(ARMS); ++k) {
            if (solution.armsSolved[k]) {
                numArmsSolved++;
            }
        }
        return numArmsSolved;
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

            // Check for self-collision with this solution
            planning_scene::PlanningScenePtr currentScene = planningScene->getPlanningScene();

            ROS_DEBUG("Received %lu results from IK query", ikQuery.response.results.size());
            for (IKList::iterator j = ikQuery.response.results.begin(); j != ikQuery.response.results.end(); ++j) {

                // We estimate that the robot will move to the position in the cache. This may be innaccurate and there
                // may be another position that is not in collision.
                // Note: The allowed collision matrix should prevent collisions between the arms
                robot_state::RobotState currentRobotState = currentScene->getCurrentState();
                currentRobotState.setVariablePositions(jointNames[j->group], j->positions);
                if (currentScene->isStateColliding(currentRobotState, j->group, true)) {
                    ROS_INFO("State in collision.");
                    continue;
                }

                possibleSolution.feasable = true;

                // We want to search for the fastest arm movement. The idea is that the first arm there should slow the human
                // down and give the second arm time to arrive.
                possibleSolution.delta = max(possibleSolution.delta, ros::Duration(i->time) - calcExecutionTime(j->group, j->positions));

                for (unsigned int k = 0; k < boost::size(ARMS); ++k) {
                    if (j->group == ARMS[k]) {
                        possibleSolution.armsSolved[k] = true;
                    }
                }
             }

             if (possibleSolution.feasable) {
                solutions.push_back(possibleSolution);
             } else {
                ROS_INFO("Solution was not feasible");
             }
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
            if (numArmsSolved(*solution) > highestArmsSolved || numArmsSolved(*solution) == highestArmsSolved && solution->delta > highestDeltaTime) {
                highestArmsSolved = numArmsSolved(*solution);
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
            if (bestSolution->armsSolved[i]) {
                humanoid_catching::Move command;
                visualizeGoal(bestSolution->pose, i);
                command.header = bestSolution->pose.header;
                command.target = bestSolution->pose.pose;
                command.obstacle = obstacle.pose;
                command.has_obstacle = false;
                command.point_at_target = true;
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
