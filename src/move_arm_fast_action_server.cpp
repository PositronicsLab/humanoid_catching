#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <human_catching/MoveArmFastAction.h>
#include <kinematics_cache/IKQuery.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

namespace {
using namespace std;
using namespace human_catching;

typedef actionlib::SimpleActionServer<human_catching::MoveArmFastAction> Server;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> JointTrajClient;

class MoveArmFastActionServer {
private:

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

    //! Action Server
    Server as;

    //! Joint trajectory action client.
    auto_ptr<JointTrajClient> jointTrajClient;

    //! Arm name
    string arm;

    //! Joint names
    vector<string> jointNames;

    //! Create messages that are used to published feedback/result
    human_catching::MoveArmFastFeedback feedback;
    human_catching::MoveArmFastResult result;
public:
	MoveArmFastActionServer(const string& name) :
		pnh("~"),
       as(nh, name, boost::bind(&MoveArmFastActionServer::execute, this, _1), false) {
        pnh.param<string>("arm", arm, "right");
        ROS_INFO("Initializing move arm fast action for arm %s", arm.c_str());

        string armActionServer = arm == "right" ? "r_arm_controller/joint_trajectory_action" :
            "l_arm_controller/joint_trajectory_action";

        nh.getParam(arm == "right" ? "/r_arm_controller/joints" : "/l_arm_controller/joints", jointNames);

        ROS_INFO("Waiting for %s", armActionServer.c_str());
        jointTrajClient.reset(new JointTrajClient(armActionServer, true));
        jointTrajClient->waitForServer();

        ROS_INFO("Starting the action server");
        as.registerPreemptCallback(boost::bind(&MoveArmFastActionServer::preempt, this));
        as.start();
        ROS_INFO("Move arm fast action initialized successfully");
	}

    private:
    /*
     * Send a goal to an action client, wait for a result, and
     * report success or failure.
     * @param client Action client to send the goal to
     * @param goal Goal to send
     * @return Whether the goal was executed successfully.
     */
    template<class T, class U>
    static bool sendGoal(const T& client, const U& goal, ros::NodeHandle& nh, double timeout = 20.0){
        bool success = false;
        if (nh.ok()){
            client->sendGoal(goal);
            if(!client->waitForResult(ros::Duration(timeout))){
                client->cancelGoal();
                ROS_INFO("Timed out achieving goal");
            }
        else {
            actionlib::SimpleClientGoalState state = client->getState();
            success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
            if(success){
                ROS_DEBUG("Action finished: %s", state.toString().c_str());
            }
            else {
                ROS_INFO("Action failed: %s", state.toString().c_str());
            }
        }
    }
    else {
        ROS_INFO("Nodehandle is invalid. Not sending action");
    }

    return success;
    }

    void preempt() {
        jointTrajClient->cancelGoal();
        as.setPreempted();
    }

    void execute(const human_catching::MoveArmFastGoalConstPtr& moveArmGoal){

        ROS_INFO("Moving arm %s to position", arm.c_str());

        if(!as.isActive() || as.isPreemptRequested() || !ros::ok()){
            ROS_INFO("Move arm action cancelled before started");
            return;
        }

        // Now use the arm trajectory client to execute it.
        pr2_controllers_msgs::JointTrajectoryGoal goal;

        // First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names = jointNames;

        if (goal.trajectory.joint_names.size() != moveArmGoal->joint_positions.size()) {
            ROS_ERROR("Incorrect number of joint positions");
            as.setAborted();
            return;
        }

        // We will have one waypoints in this goal trajectory
        goal.trajectory.points.resize(1);
        goal.trajectory.points[0].positions = moveArmGoal->joint_positions;
        // TODO: Confirm this waypoint is correct
        goal.trajectory.points[0].time_from_start = ros::Duration(0.1);
        goal.trajectory.header.stamp = ros::Time::now();
        bool success = sendGoal(jointTrajClient.get(), goal, nh);

        if(as.isPreemptRequested() || !ros::ok()){
            ROS_INFO("Action was preempted for arm %s", arm.c_str());
            as.setPreempted();
        } else if (success) {
            ROS_INFO("Move arm fast succeeded for arm %s", arm.c_str());
            as.setSucceeded(result);
        } else {
            ROS_INFO("Move arm fast failed for arm %s", arm.c_str());
            as.setAborted(result);
        }
    }
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "move_arm_fast_action");

	MoveArmFastActionServer maf(ros::this_node::getName());
	ros::spin();
}
