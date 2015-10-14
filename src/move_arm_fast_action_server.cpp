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
    
    //! Cached service client.
    ros::ServiceClient ikService;
    
    //! Joint trajectory action client.
    JointTrajClient jointTrajClient;
    
    //! Create messages that are used to published feedback/result
    human_catching::MoveArmFastFeedback feedback;
    human_catching::MoveArmFastResult result;  
public:
	MoveArmFastActionServer() :
		pnh("~"),
       as(nh, "move_arm_fast_action", boost::bind(&MoveArmFastActionServer::execute, this, _1), false),
       // TODO: Use right and left clients?
       jointTrajClient("r_arm_controller/joint_trajectory_action", true) {
        
        ROS_INFO("Initializing move arm fast action");
        ros::service::waitForService("/kinematics_cache/ik");
        ikService = nh.serviceClient<kinematics_cache::IKQuery>("/kinematics_cache/ik", true /* persistent */);
        jointTrajClient.waitForServer();
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
                ROS_INFO("Action failed: %s",state.toString().c_str());
            }   
        }
    }
    else {
        ROS_INFO("Nodehandle is invalid. Not sending action");
    }

    return success;
    }

    void preempt() {
        jointTrajClient.cancelGoal();
        as.setPreempted();
    }
    
    void execute(const human_catching::MoveArmFastGoalConstPtr& moveArmGoal){
        
        ROS_INFO("Moving to position");
    
        if(!as.isActive() || as.isPreemptRequested() || !ros::ok()){
            ROS_INFO("Move arm action cancelled before started");
            return;
        }
        
        // Lookup the IK solution
        kinematics_cache::IKQuery ikQuery;
        ikQuery.request.group = moveArmGoal->group;
        ikQuery.request.pose = moveArmGoal->target;
        if (!ikService.call(ikQuery)) {
            ROS_WARN("Failed to find IK solution for fast arm movement");
            as.setAborted();
            return;
        }
        
        // Now use the arm trajectory client to execute it.
        pr2_controllers_msgs::JointTrajectoryGoal goal;

        // First, the joint names, which apply to all waypoints
        // TODO: Fetch this from the model
        goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

        // We will have one waypoints in this goal trajectory
        goal.trajectory.points.resize(1);
        goal.trajectory.points[0].positions = ikQuery.response.positions;
        // TODO: Confirm this waypoint is correct
        goal.trajectory.points[0].time_from_start = ros::Duration(0.0);
        goal.trajectory.header.stamp = ros::Time::now();
        sendGoal(&jointTrajClient, goal, nh);
        
        if(as.isPreemptRequested() || !ros::ok()){
            as.setPreempted();
        }
        else {
            as.setSucceeded(result);
        }
    }
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "move_arm_fast_action");

	MoveArmFastActionServer maf;
	ros::spin();
}
