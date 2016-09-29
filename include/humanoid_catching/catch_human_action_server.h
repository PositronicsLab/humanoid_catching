#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <humanoid_catching/CatchHumanAction.h>
#include <kinematics_cache/IKQueryv2.h>
#include <humanoid_catching/PredictFall.h>
#include <actionlib/client/simple_action_client.h>
#include <humanoid_catching/CalculateTorques.h>
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
#include <Ravelin/URDFReaderd.h>
#include <Ravelin/RigidBodyd.h>
#include <Ravelin/Jointd.h>
#include <Ravelin/RCArticulatedBodyd.h>
#include <operational_space_controllers_msgs/Move.h>
#include <geometry_msgs/WrenchStamped.h>
#include <kinematics_cache/kinematics_cache.h>

typedef actionlib::SimpleActionServer<humanoid_catching::CatchHumanAction> Server;
typedef std::vector<kinematics_cache::IKv2> IKList;

struct Limits
{
    Limits() : velocity(0.0), acceleration(0.0), effort(0.0) {}
    Limits(double aVelocity, double aAcceleration, double aEffort) : velocity(aVelocity), acceleration(aAcceleration), effort(aEffort) {}
    double velocity;
    double acceleration;
    double effort;
};

typedef std::map<std::string, Limits> LimitMapType;

struct State
{
    State() : position(0.0), velocity(0.0) {}
    State(double aPosition, double aVelocity) : position(aPosition), velocity(aVelocity) {}
    double position;
    double velocity;
};

typedef std::map<std::string, State> StateMapType;

struct Solution
{
    geometry_msgs::PointStamped position;
    geometry_msgs::PoseStamped targetPose;
    geometry_msgs::TwistStamped targetVelocity;
    ros::Duration delta;
    ros::Duration time;
};

class CatchHumanActionServer
{
private:

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Action Server
    Server as;

    //! Cached fall prediction client
    ros::ServiceClient fallPredictor;

    //! IK Cache
    std::auto_ptr<kinematics_cache::KinematicsCache> ik;

    //! Cached balancing client
    ros::ServiceClient balancer;

    //! Subscriber for joint state updates
    std::auto_ptr<message_filters::Subscriber<sensor_msgs::JointState> > jointStatesSub;

    //! Arm client
    ros::Publisher armCommandPub;

    //! TF listener
    tf::TransformListener tf;

    //! Visualization of goals
    ros::Publisher goalPub;

    //! Visualization of velocities
    ros::Publisher eeVelocityVizPub;

    //! Visualization of trials
    ros::Publisher trialGoalPub;

    //! Visualization of target pose
    ros::Publisher targetPosePub;

    //! Visualization of target velocity
    ros::Publisher targetVelocityPub;

    //! Current joint states
    StateMapType jointStates;

    //! Joint limits
    LimitMapType jointLimits;

    //! Joint names
    std::vector<std::string> jointNames;

    //! Planning scene
    boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planningScene;

    //! Kinematic model of the robot
    robot_model::RobotModelPtr kinematicModel;

    //! Ravelin dynamic body
    boost::shared_ptr<Ravelin::RCArticulatedBodyd> body;

    //! Record the indexes into the body for the two sets of arm joints
    std::map<std::string, int> jointIndices;

    //! Arm name
    std::string arm;

    //! EE Frame
    std::string eeFrame;

    //! Create messages that are used to published feedback/result
    humanoid_catching::CatchHumanFeedback feedback;
    humanoid_catching::CatchHumanResult result;

public:
    CatchHumanActionServer(const std::string& name);
    static double calcJointExecutionTime(const Limits& limits, const double signed_d, double v0);
private:

#if ROS_VERSION_MINIMUM(1, 10, 12)
    // Method not required
#else
    static std::vector<std::string> getActiveJointModelNames(const robot_model::JointModelGroup* jointModelGroup);
#endif // ROS_VERSION_MINIMUM

    void preempt();
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void visualizeGoal(const geometry_msgs::Pose& goal, const std_msgs::Header& header,
                       geometry_msgs::PoseStamped targetPose, geometry_msgs::TwistStamped targetVelocity,
                       double humanoidRadius, double humanoidHeight) const;

    static geometry_msgs::PointStamped poseToPoint(const geometry_msgs::PoseStamped pose);

    ros::Duration calcExecutionTime(const std::vector<double>& solution);
    static geometry_msgs::Pose applyTransform(const geometry_msgs::PoseStamped& pose, const tf::StampedTransform transform);
    static geometry_msgs::Vector3 applyTransform(const geometry_msgs::Vector3& linear, const tf::StampedTransform transform);

    geometry_msgs::Pose tfFrameToPose(const std::string& tfFrame, const ros::Time& stamp, const std::string& base) const;

    const std::vector<humanoid_catching::FallPoint>::const_iterator findContact(const humanoid_catching::PredictFall::Response& fall) const;

    geometry_msgs::Pose endEffectorPosition(const std::string& frame) const;

    void visualizeEEVelocity(const std::vector<double>& eeVelocity);

    void sendTorques(const std::vector<double>& torques);

    void updateRavelinModel();

    geometry_msgs::Twist linkVelocity(const std::string& linkName);

    bool predictFall(const humanoid_catching::CatchHumanGoalConstPtr& human, humanoid_catching::PredictFall& predictFall,
                     ros::Duration duration, bool includeEndEffectors, bool visualize);

    void execute(const humanoid_catching::CatchHumanGoalConstPtr& goal);

public:
    static geometry_msgs::Quaternion computeOrientation(const Solution& solution, const geometry_msgs::Pose& currentPose);
};
