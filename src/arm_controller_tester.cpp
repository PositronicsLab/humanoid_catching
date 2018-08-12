#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group.h>
#include <control_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <humanoid_catching/MovementGoal.h>
#include <humanoid_catching/DurationStamped.h>

using namespace std;
typedef actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction> TrajClient;

class ArmControllerTester {
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    auto_ptr<message_filters::Subscriber<humanoid_catching::MovementGoal> > movementGoalSub;
    ros::Publisher movementGoalMetricsPub;
    ros::Publisher movementPlanMetricsPub;

    string endEffector;
    string arm;
    string commandTopic;

    robot_model::JointModelGroup* jointModelGroup;
    robot_state::RobotStatePtr kinematicState;
    auto_ptr<TrajClient> trajectoryClient;
    auto_ptr<moveit::planning_interface::MoveGroup> moveGroup;
public:
    ArmControllerTester() :
            pnh("~")
        {

        if (!pnh.getParam("end_effector", endEffector))
        {
            ROS_ERROR("End effector name must be specified");
        }
        else {
            ROS_INFO("Configuring for end effector: [%s]", endEffector.c_str());
        }

        if (!pnh.getParam("arm", arm))
        {
            ROS_ERROR("Arm name must be specified");
        }
        else {
            ROS_INFO("Configuring for arm: [%s]", arm.c_str());
        }

        if (!pnh.getParam("command_topic", commandTopic))
        {
            ROS_ERROR("Command topic must be specified");
        }
        else {
            ROS_INFO("Subscribing to topic: [%s]", commandTopic.c_str());
        }

        // Setup the subscriber
        movementGoalSub.reset(new message_filters::Subscriber<humanoid_catching::MovementGoal>(nh,commandTopic, 1000));
        movementGoalSub->registerCallback(boost::bind(&ArmControllerTester::goalCallback, this, _1));

        // Setup the publisher
        movementGoalMetricsPub = nh.advertise<humanoid_catching::DurationStamped>("movement_execute_metric", 100000, true);
        movementPlanMetricsPub = nh.advertise<humanoid_catching::DurationStamped>("movement_plan_metric", 100000, true);

        robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
        robot_model::RobotModelPtr kinematicModel = robotModelLoader.getModel();

        kinematicState.reset(new robot_state::RobotState(kinematicModel));
        kinematicState->setToDefaultValues();
        jointModelGroup = kinematicModel->getJointModelGroup(arm);

        trajectoryClient.reset(new TrajClient(arm == "left" ? "l_arm_controller/joint_trajectory_action" : "r_arm_controller/joint_trajectory_action", true));

        ROS_INFO("Waiting for the joint_trajectory_action server");
        if(!trajectoryClient->waitForServer(ros::Duration(30))){
            ROS_ERROR("joint trajectory server could not be found");
        }
        ROS_INFO("joint_trajectory_action server is up");

        moveGroup.reset(new moveit::planning_interface::MoveGroup(arm));
    }

private:

    static double distance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
        return sqrt(pow(pose2.position.x - pose1.position.x, 2) + pow(pose2.position.y - pose1.position.y, 2) + pow(pose2.position.z - pose1.position.z, 2));
    }

    void goalCallback(const humanoid_catching::MovementGoalConstPtr movementGoal) {

        ROS_INFO("Receive a goal callback");

        setArmToStart(movementGoal->jointAngles);

        moveGroup->setGoalPositionTolerance(0.01);
        moveGroup->setGoalOrientationTolerance(0.01);
        moveGroup->setPoseTarget(movementGoal->pose);
        moveGroup->setPlanningTime(1.0);
        moveGroup->allowLooking(false);
        moveGroup->allowReplanning(false);

        ros::WallTime startMove = ros::WallTime::now();
        ros::Time startMoveRos = ros::Time::now();
        if (moveGroup->asyncMove() == -1) {
            ROS_WARN("Failed to start movement");
            return;
        }

        ROS_INFO("Polling for result");
        bool succeeded = false;
        while (ros::Duration(0.001).sleep() && ros::Time::now() - startMoveRos < ros::Duration(5)) {
            const geometry_msgs::PoseStamped& pose = moveGroup->getCurrentPose();
            ROS_INFO("Current position [%f %f %f]", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            ROS_INFO("Desired position [%f %f %f]", movementGoal->pose.position.x, movementGoal->pose.position.y, movementGoal->pose.position.z);
            double current = distance(pose.pose, movementGoal->pose);
            ROS_INFO("Current distance: [%f]", current);
            // TODO: Add orientation
            if (current <= 0.01) {
                ROS_INFO("Reached goal");
                succeeded = true;
                break;
            }
        }
        ROS_INFO("Finished polling for result");

        if (!succeeded) {
            ROS_WARN("Failed to reach goal");
            return;
        }
        ROS_INFO("Succeeded!");
        {
            humanoid_catching::DurationStamped msg;
            msg.arm = arm;
            msg.header = movementGoal->header;
            msg.duration = ros::Duration((ros::WallTime::now() - startMove).toSec());
            ROS_INFO("Duration of move was [%f]", msg.duration.toSec());
            movementGoalMetricsPub.publish(msg);
        }

        ROS_INFO("Exiting goal callback");
    }

    void setArmToStart(const vector<double>& positions) {

        control_msgs::JointTrajectoryGoal goal;
        goal.trajectory.header.stamp = ros::Time::now();

        if (arm == "left") {
            goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
            goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
            goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
            goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
            goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
            goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
            goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
        }
        else {
            goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
            goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
            goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
            goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
            goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
            goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
            goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
        }
        goal.trajectory.points.resize(1);
        goal.trajectory.points[0].positions = positions;

        goal.trajectory.points[0].velocities.resize(7);
        goal.trajectory.points[0].velocities[0] = 0.0;
        goal.trajectory.points[0].time_from_start = ros::Duration(0.5);

        trajectoryClient->sendGoal(goal);
        ROS_INFO("Sent goal to reset arm positions");

        // Wait for the action to return
        if (trajectoryClient->waitForResult(ros::Duration(30.0))) {
            actionlib::SimpleClientGoalState state = trajectoryClient->getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        }
        else {
            ROS_WARN("Failed to move to base position");
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_controller_tester");
    ArmControllerTester act;
    ros::spin();
    return 0;
}

