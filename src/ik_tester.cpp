#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <humanoid_catching/IKMetric.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf/transform_listener.h>
#include <pluginlib/class_loader.h>

using namespace std;

class IKTester {
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    auto_ptr<message_filters::Subscriber<geometry_msgs::PointStamped> > trialGoalSub;
    kinematics::KinematicsBasePtr kinematicsSolver;
    ros::Publisher ikMetricsPub;

    string endEffector;
    string arm;
    string commandTopic;

    robot_model::JointModelGroup* jointModelGroup;
    robot_state::RobotStatePtr kinematicState;
    boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematicsLoader;
public:
    IKTester() :
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
        trialGoalSub.reset(
                new message_filters::Subscriber<geometry_msgs::PointStamped>(nh,
                        commandTopic, 10000));
        trialGoalSub->registerCallback(boost::bind(&IKTester::goalCallback, this, _1));

        // Setup the publisher
        ikMetricsPub = nh.advertise<humanoid_catching::IKMetric>("ik_metric", 1, true);

        robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
        robot_model::RobotModelPtr kinematicModel = robotModelLoader.getModel();
        ROS_INFO("Model frame: %s", kinematicModel->getModelFrame().c_str());

        kinematicState.reset(new robot_state::RobotState(kinematicModel));
        kinematicState->setToDefaultValues();
        jointModelGroup = kinematicModel->getJointModelGroup(arm);

        kinematicsLoader.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));
        try {
            kinematicsSolver = kinematicsLoader->createInstance("pr2_arm_kinematics/PR2ArmKinematicsPlugin");
        } catch(pluginlib::PluginlibException& ex) //handle the class failing to load
        {
            ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
            throw ex;
        }

        if(!kinematicsSolver->initialize("robot_description", arm, "/torso_lift_link", endEffector, 0.01)) {
            ROS_ERROR("Could not initialize solver");
        }
        else {
            ROS_INFO("Initialized solver successfully");
        }
    }

private:

    void goalCallback(const geometry_msgs::PointStampedConstPtr& point) {

        ROS_INFO("Querying for solution");

        ros::Time start = ros::Time::now();

        // Create a pose in the default orientation.
        geometry_msgs::Pose pose;
        pose.position = point->point;
        tf::Quaternion identity = tf::createIdentityQuaternion();
        tf::quaternionTFToMsg(identity, pose.orientation);

        vector<double> jointValues(jointModelGroup->getJointModels().size());
        for (unsigned int i = 0; i < jointValues.size(); ++i) {
            jointValues[i] = (jointModelGroup->getJointModels()[i]->getVariableBounds()[0].first
                + jointModelGroup->getJointModels()[i]->getVariableBounds()[0].second ) / 2.0;
            ROS_DEBUG("Set initial position to [%f]", jointValues[i]);
        }

        vector<double> solution(7);
        moveit_msgs::MoveItErrorCodes errorCode;

        bool success = false;
        kinematicsSolver->searchPositionIK(pose, jointValues, 0.1, solution, errorCode);
        if(errorCode.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_INFO("Solution found");
            success = true;
        }
        else {
            ROS_WARN("IK failed %i", errorCode.val);
        }

        humanoid_catching::IKMetric msg;
        msg.time = ros::Time::now() - start;
        msg.was_successful = success;
        msg.distance = 0.0;
        ikMetricsPub.publish(msg);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ik_tester");
    IKTester ik;
    ros::spin();
    return 0;
}

