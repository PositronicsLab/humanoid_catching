#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <humanoid_catching/CatchHumanAction.h>
#include <kinematics_cache/IKQueryv2.h>
#include <humanoid_catching/PredictFall.h>
#include <actionlib/client/simple_action_client.h>
#include <humanoid_catching/CalculateTorques.h>
#include <humanoid_catching/IKMetric.h>
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
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <map>
#include <std_srvs/Empty.h>

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
    ros::Duration estimate;
    double height;
    double distance;
};

class CatchHumanController
{
private:

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Whether the controller is active
    bool active;

    //! Cached create mesh cache client
    ros::ServiceClient createMeshCache;

    //! Cached fall prediction client
    ros::ServiceClient fallPredictor;

    //! Cached last sensor reading
    sensor_msgs::ImuConstPtr cachedImuData;

    //! IK Cache
    std::auto_ptr<kinematics_cache::KinematicsCache> ik;

    //! Cached balancing client
    ros::ServiceClient balancer;

    //! Subscriber for joint state updates
    std::auto_ptr<message_filters::Subscriber<sensor_msgs::JointState> > jointStatesSub;

    //! Arm client
    ros::Publisher armCommandPub;

    //! Visualization of kinematic cache state for end effectors
    ros::Publisher ikCachePub;

    //! Visualization of goals
    ros::Publisher goalPub;

    //! Record goals
    ros::Publisher movementGoalPub;

    //! Visualization of goals as points
    ros::Publisher goalPointPub;

    //! Visualization of velocities
    ros::Publisher eeVelocityVizPub;

    //! Visualization of trials
    ros::Publisher trialGoalPub;

    //! Visualization of target pose
    ros::Publisher targetPosePub;

    //! Visualization of target velocity
    ros::Publisher targetVelocityPub;

    //! Publisher of reaction time
    ros::Publisher reactionTimePub;

    //! Publisher of reaction time
    ros::Publisher interceptTimePub;

    //! Publisher of fall prediction time
    ros::Publisher fallPredictionTimePub;

    //! Publisher of fall prediction time
    ros::Publisher balancingTimePub;

    //! Publisher of IK metrics
    ros::Publisher ikMetricsPub;

    //! Empty service to indicate the node is ready
    ros::ServiceServer readyService;

    //! Current joint states
    StateMapType jointStates;

    //! Joint limits
    LimitMapType jointLimits;

    //! Constant transform from world to base frame
    tf::StampedTransform goalToBaseTransform;

    //! Global frame
    std::string globalFrame;

    //! Base frame
    std::string baseFrame;

    //! Queue for the joint state messages
    ros::CallbackQueue jointStateMessagesQueue;

    //! Joint states lock
    mutable boost::shared_mutex jointStatesAccess;

    //! Spinner
    std::auto_ptr<ros::AsyncSpinner> jointStateMessagesSpinner;

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

    //! Contact time tolerance
    ros::Duration contactTimeTolerance;

    //! Human fall subscriber
    std::auto_ptr<message_filters::Subscriber<std_msgs::Header> > humanFallSub;

    //! Reset sub
    std::auto_ptr<message_filters::Subscriber<std_msgs::Header> > resetSub;

    //! Human IMU subscriber
    std::auto_ptr<message_filters::Subscriber<sensor_msgs::Imu> > humanIMUSub;

    //! All arm links
    std::vector<const robot_model::LinkModel*> allArmLinks;

    //! URDF Model
    boost::shared_ptr<const urdf::ModelInterface> urdfModel;
public:
    CatchHumanController();
    ~CatchHumanController();
    static double calcJointExecutionTime(const Limits& limits, const double signed_d, double v0);
private:
    unsigned int countJointsAboveInChain(const robot_model::LinkModel* link) const;
    const robot_model::LinkModel* findParentLinkInJointModelGroup(const robot_model::LinkModel* link) const;
    const robot_model::JointModel* findParentActiveJoint(const robot_model::LinkModel* start) const;
    bool noop(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void publishIkCache();

#if ROS_VERSION_MINIMUM(1, 10, 12)
    // Method not required
#else
    static std::vector<std::string> getActiveJointModelNames(const robot_model::JointModelGroup* jointModelGroup);
#endif // ROS_VERSION_MINIMUM
    void fallDetected(const std_msgs::HeaderConstPtr& fallingMsg);
    void imuDataDetected(const sensor_msgs::ImuConstPtr& imuData);
    void reset(const std_msgs::HeaderConstPtr& reset);
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void visualizeGoal(const geometry_msgs::Pose& goal, const std_msgs::Header& header,
                       geometry_msgs::PoseStamped targetPose, geometry_msgs::TwistStamped targetVelocity,
                       double humanoidRadius, double humanoidHeight) const;

    static geometry_msgs::PointStamped poseToPoint(const geometry_msgs::PoseStamped pose);

    ros::Duration calcExecutionTime(const std::vector<double>& solution) const;
    geometry_msgs::PoseStamped transformGoalToBase(const geometry_msgs::PoseStamped& pose) const;
    geometry_msgs::PoseStamped transformBaseToGoal(const geometry_msgs::PoseStamped& pose) const;
    geometry_msgs::Vector3 transformGoalToBase(const geometry_msgs::Vector3& linear) const;

    const std::vector<humanoid_catching::FallPoint>::const_iterator findContact(const humanoid_catching::PredictFall::Response& fall) const;

    bool linkPosition(const std::string& linkName, geometry_msgs::PoseStamped& pose, const robot_state::RobotState& currentRobotState) const;

    void visualizeEEVelocity(const std::vector<double>& eeVelocity);

    void sendTorques(const std::vector<double>& torques);

    void updateRavelinModel();

    bool predictFall(const sensor_msgs::ImuConstPtr imuData, humanoid_catching::PredictFall& predictFall,
                     ros::Duration duration);

    bool initMeshCache();

    void execute(const sensor_msgs::ImuConstPtr imuData);

    void calcArmLinks();

    bool checkFeasibility(const humanoid_catching::FallPoint& fallPoint, Solution& possibleSolution, const std_msgs::Header& fallPointHeader) const;

public:
    static geometry_msgs::Quaternion computeOrientation(const Solution& solution, const geometry_msgs::PoseStamped& currentPose);
};
