#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <humanoid_catching/CatchHumanAction.h>
#include <kinematics_cache/IKQueryv2.h>
#include <humanoid_catching/PredictFall.h>
#include <humanoid_catching/DurationStamped.h>
#include <humanoid_catching/CreateMeshCache.h>
#include <humanoid_catching/catch_human_controller.h>
#include <humanoid_catching/MovementGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <humanoid_catching/CalculateTorques.h>
#include <tf/transform_listener.h>
#include <boost/timer.hpp>
#include <boost/math/constants/constants.hpp>
#include <map>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/link_model.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/link_state.h>
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
#include <ros/spinner.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/SolidPrimitive.h>
#include <std_msgs/Duration.h>

using namespace std;
using namespace humanoid_catching;

static const double MAX_VELOCITY = 100;
static const double MAX_ACCELERATION = 100;
static const double MAX_EFFORT = 100;

//! Tolerance of time to be considered in contact
static const ros::Duration CONTACT_TIME_TOLERANCE_DEFAULT = ros::Duration(0.01);
static const ros::Duration MAX_DURATION = ros::Duration(1.0);
static const ros::Duration STEP_SIZE = ros::Duration(0.001);
static const ros::Duration MAX_GRID_SEARCH_DURATION = ros::Duration(0.004);
static const ros::Duration SEARCH_RESOLUTION(0.03);
static const double pi = boost::math::constants::pi<double>();

#define ENABLE_EXECUTION_TIME_DEBUGGING 0

static tf::Quaternion quaternionFromVector(const tf::Vector3& v)
{
    tf::Vector3 u(1.0, 0.0, 0.0);

    if (u.x() == v.x() && u.y() == v.y() && u.z() == v.z())
    {
        return tf::Quaternion::getIdentity();
    }

    float cos_theta = u.normalized().dot(v.normalized());
    float angle = acos(cos_theta);
    tf::Vector3 w = u.normalized().cross(v.normalized()).normalized();
    return tf::Quaternion(w, angle).normalized();
}

static tf::Vector3 quatToVector(const tf::Quaternion& orientation)
{
    tf::Transform rotation(orientation);
    tf::Vector3 xAxis(1, 0, 0);
    tf::Vector3 r = rotation.getBasis() * xAxis;
    r.normalize();
    return r;
}

geometry_msgs::PoseStamped CatchHumanController::transformGoalToBase(const geometry_msgs::PoseStamped& pose) const
{
    assert(pose.header.frame_id == globalFrame);
    assert(goalToBaseTransform.child_frame_id_ == globalFrame);
    assert(goalToBaseTransform.frame_id_ == baseFrame);

    tf::Stamped<tf::Pose> tfPose;
    tf::poseStampedMsgToTF(pose, tfPose);
    tf::Pose tfGoal = goalToBaseTransform * tfPose;
    geometry_msgs::PoseStamped msgGoal;
    tf::poseTFToMsg(tfGoal, msgGoal.pose);
    msgGoal.header.stamp = ros::Time::now();
    msgGoal.header.frame_id = baseFrame;
    return msgGoal;
}

geometry_msgs::PoseStamped CatchHumanController::transformBaseToGoal(const geometry_msgs::PoseStamped& pose) const
{
    assert(pose.header.frame_id == baseFrame);
    assert(goalToBaseTransform.child_frame_id_ == globalFrame);
    assert(goalToBaseTransform.frame_id_ == baseFrame);

    tf::Stamped<tf::Pose> tfPose;
    tf::poseStampedMsgToTF(pose, tfPose);
    tf::Pose tfGoal = goalToBaseTransform.inverse() * tfPose;
    geometry_msgs::PoseStamped msgGoal;
    tf::poseTFToMsg(tfGoal, msgGoal.pose);
    msgGoal.header.stamp = ros::Time::now();
    msgGoal.header.frame_id = globalFrame;
    return msgGoal;
}

geometry_msgs::Vector3 CatchHumanController::transformGoalToBase(const geometry_msgs::Vector3& linear) const
{
    assert(goalToBaseTransform.child_frame_id_ == globalFrame);
    assert(goalToBaseTransform.frame_id_ == baseFrame);

    tf::Stamped<tf::Vector3> tfLinear;

    tf::vector3MsgToTF(linear, tfLinear);
    tf::Vector3 tfGoal = goalToBaseTransform * tfLinear;

    geometry_msgs::Vector3 msgGoal;
    tf::vector3TFToMsg(tfGoal, msgGoal);
    return msgGoal;
}

void CatchHumanController::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    ROS_DEBUG("Updating joint states");

    // Take read lock for the entire update.
    boost::unique_lock<boost::shared_mutex> lock(jointStatesAccess);
    for(unsigned int i = 0; i < msg->name.size(); ++i)
    {
        jointStates[msg->name[i]] = State(msg->position[i], msg->velocity[i]);
    }
    ROS_DEBUG("Updated joint states");
}

CatchHumanController::CatchHumanController() :
    pnh("~"),
    planningScene(new planning_scene_monitor::PlanningSceneMonitor("robot_description")),
    active(false){
    ROS_INFO("Initializing the catch human action");

    // Configure the planning scene
    planningScene->startStateMonitor();
    planningScene->setStateUpdateFrequency(100); // 100hz

    // Configure the fall prediction service
    string fallPredictorService;
    if (!pnh.getParam("fall_predictor", fallPredictorService))
    {
        ROS_ERROR("fall_predictor must be specified");
    }
    ROS_DEBUG("Waiting for %s service", fallPredictorService.c_str());
    ros::service::waitForService(fallPredictorService);
    fallPredictor = nh.serviceClient<humanoid_catching::PredictFall>(fallPredictorService, true /* persistent */);

    // Configure the mesh creation service
    string createMeshCacheService;
    if (!pnh.getParam("create_mesh_cache", createMeshCacheService))
    {
        ROS_ERROR("create_mesh_cache must be specified");
    }
    ROS_DEBUG("Waiting for %s service", createMeshCacheService.c_str());
    ros::service::waitForService(createMeshCacheService);
    createMeshCache = nh.serviceClient<humanoid_catching::CreateMeshCache>(createMeshCacheService, true /* persistent */);

    double maxDistance;
    pnh.param("max_distance", maxDistance, 0.821000);

    {
        double contactTimeToleranceDouble;
        pnh.param("contact_time_tolerance", contactTimeToleranceDouble, CONTACT_TIME_TOLERANCE_DEFAULT.toSec());
        contactTimeTolerance = ros::Duration(contactTimeToleranceDouble);
    }

    pnh.param<string>("base_frame", baseFrame, "/torso_lift_link");
    pnh.param<string>("global_frame", globalFrame, "/odom_combined");

    if (!pnh.getParam("arm", arm))
    {
        ROS_ERROR("Arm name must be specified");
    }

    string armCommandTopic;
    if (!pnh.getParam("command_topic", armCommandTopic))
    {
        ROS_ERROR("command_topic must be specified");
    }

    string cacheDataName;
    if (!pnh.getParam("cache_data_name", cacheDataName))
    {
        ROS_ERROR("cache_data_name must be specified");
    }

    humanFallSub.reset(
        new message_filters::Subscriber<std_msgs::Header>(nh, "/human/fall", 1));
    humanFallSub->registerCallback(boost::bind(&CatchHumanController::fallDetected, this, _1));

    resetSub.reset(
        new message_filters::Subscriber<std_msgs::Header>(nh, "/catching_controller/reset", 1));
    resetSub->registerCallback(boost::bind(&CatchHumanController::reset, this, _1));

    humanIMUSub.reset(
        new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/in", 1));
    humanIMUSub->registerCallback(boost::bind(&CatchHumanController::imuDataDetected, this, _1));

    ik.reset(new kinematics_cache::KinematicsCache(maxDistance, baseFrame, cacheDataName));

    string balancerService;
    if (!pnh.getParam("balancer", balancerService))
    {
        ROS_ERROR("balancer must be specified");
    }
    ROS_DEBUG("Waiting for %s service", balancerService.c_str());
    ros::service::waitForService(balancerService);
    balancer = nh.serviceClient<humanoid_catching::CalculateTorques>(balancerService, true /* persistent */);

    // Initialize arm clients
    ROS_DEBUG("Initializing arm command publisher");
    armCommandPub = nh.advertise<operational_space_controllers_msgs::Move>(armCommandTopic, 1, false);

    // Initialize visualization publishers
    goalPub = pnh.advertise<geometry_msgs::PoseStamped>("movement_goal", 10, true);
    movementGoalPub = pnh.advertise<humanoid_catching::MovementGoal>("record_movement_goal", 100, true);
    goalPointPub = pnh.advertise<geometry_msgs::PointStamped>("movement_goal_point", 10, true);
    targetPosePub = pnh.advertise<geometry_msgs::PoseStamped>("target_pose", 1);
    targetVelocityPub = pnh.advertise<visualization_msgs::Marker>("target_velocity", 1);
    trialGoalPub = pnh.advertise<geometry_msgs::PointStamped>("movement_goal_trials", 1);
    eeVelocityVizPub = pnh.advertise<geometry_msgs::WrenchStamped>("ee_velocity", 1);

    // Metrics publishers
    reactionTimePub = nh.advertise<humanoid_catching::DurationStamped>("reaction_time", 1, true);
    balancingTimePub = pnh.advertise<humanoid_catching::DurationStamped>("balance_calc_time", 1, true);
    interceptTimePub = pnh.advertise<humanoid_catching::DurationStamped>("intercept_calc_time", 1, true);
    fallPredictionTimePub = nh.advertise<humanoid_catching::DurationStamped>("fall_predict_time", 1, true);
    ikMetricsPub = nh.advertise<humanoid_catching::IKMetric>("ik_metric", 1, true);

    ros::SubscriberStatusCallback connectCB = boost::bind(&CatchHumanController::publishIkCache, this);
    ikCachePub = pnh.advertise<visualization_msgs::MarkerArray>("ik_cache", 1, connectCB);

    rdf_loader::RDFLoader rdfLoader;
    const boost::shared_ptr<const srdf::Model> srdf = rdfLoader.getSRDF();
    urdfModel = rdfLoader.getURDF();

    kinematicModel.reset(new robot_model::RobotModel(urdfModel, srdf));
    ROS_DEBUG("Robot model initialized successfully");

    string urdfLocation;
    pnh.param<string>("urdf", urdfLocation, "pr2.urdf");

    string robotName;
    vector<boost::shared_ptr<Ravelin::RigidBodyd> > links;
    vector<boost::shared_ptr<Ravelin::Jointd> > joints;
    if (!Ravelin::URDFReaderd::read(urdfLocation, robotName, links, joints))
    {
        ROS_ERROR("Failed to parse URDF: %s", urdfLocation.c_str());
    }

    body = boost::shared_ptr<Ravelin::RCArticulatedBodyd>(new Ravelin::RCArticulatedBodyd());
    body->set_floating_base(false);
    body->set_links_and_joints(links, joints);

    // Determine the indexes for the arms within the body joints in Ravelin
    unsigned int curr = 0;
    for (unsigned int l = 0; l < joints.size(); ++l)
    {
        // Only record joints that represent generalized coordinates
        if (joints[l]->num_dof() > 0)
        {
            jointIndices[joints[l]->joint_id] = curr;
            ROS_DEBUG("Adding joint index [%u] for joint [%s]", curr, joints[l]->joint_id.c_str());
        }
        curr += joints[l]->num_dof();
    }

    // assert(curr == body->num_generalized_coordinates(Ravelin::DynamicBodyd::eEuler));

    ROS_DEBUG("Loading joint limits for arm %s", arm.c_str());
    const robot_model::JointModelGroup* jointModelGroup =  kinematicModel->getJointModelGroup(arm);

#if ROS_VERSION_MINIMUM(1, 10, 12)
    const vector<string> jointModelNames = jointModelGroup->getActiveJointModelNames();
#else
    const vector<string> jointModelNames = getActiveJointModelNames(jointModelGroup);
#endif

    for (unsigned int i = 0; i < jointModelNames.size(); ++i)
    {
        jointNames.push_back(jointModelNames[i]);
        const urdf::Joint* ujoint = urdfModel->getJoint(jointModelNames[i]).get();

        const string prefix = "robot_description_planning/joint_limits/" + jointModelNames[i] + "/";
        ROS_DEBUG_NAMED("catch_human_action_server", "Loading velocity and acceleration limits for joint %s", jointModelNames[i].c_str());

        double max_velocity;
        if (ujoint != NULL && ujoint->limits)
        {
            max_velocity = ujoint->limits->velocity;
            ROS_DEBUG_NAMED("catch_human_action_server", "Setting max velocity for joint [%s] to [%f]", jointModelNames[i].c_str(), max_velocity);
        }
        else
        {
            ROS_WARN_NAMED("catch_human_action_server", "Setting max velocity for joint [%s] to default [%f]", jointModelNames[i].c_str(), MAX_VELOCITY);
            max_velocity = MAX_VELOCITY;
        }

        bool has_acc_limits;
        double max_acc;
        if (nh.getParam(prefix + "has_acceleration_limits", has_acc_limits) && has_acc_limits && nh.getParam(prefix + "max_acceleration", max_acc))
        {
            // max accelerations are very conservative and not consistent with in-situ observations
            // TODO: Reassess this
            max_acc *= 2.0;
            ROS_DEBUG_NAMED("catch_human_action_server", "Setting max acceleration for joint [%s] to [%f]", jointModelNames[i].c_str(), max_acc);
        }
        else
        {
            ROS_WARN_NAMED("catch_human_action_server", "Setting max acceleration for joint [%s] to default [%f]", jointModelNames[i].c_str(), MAX_ACCELERATION);
            max_acc = MAX_ACCELERATION;
        }

        // Fetch the effort from the urdf
        double max_effort;
        if (ujoint != NULL && ujoint->limits)
        {
            max_effort = ujoint->limits->effort;
            ROS_DEBUG_NAMED("catch_human_action_server", "Setting max effort to %f for %s", max_effort, jointModelNames[i].c_str());
        }
        else
        {
            ROS_WARN_NAMED("catch_human_action_server", "Setting max effort to default for %s", jointModelNames[i].c_str());
            max_effort = MAX_EFFORT;
        }
        jointLimits[jointModelNames[i]] = Limits(max_velocity, max_acc, max_effort);
    }

    ROS_DEBUG("Completed initializing the joint limits");

    calcArmLinks();

    tf::TransformListener tf;
    if (!tf.waitForTransform(baseFrame, globalFrame, ros::Time(0), ros::Duration(15)))
    {
        ROS_ERROR("Failed to lookup transform from %s to %s", globalFrame.c_str(), baseFrame.c_str());
    }
    tf.lookupTransform(baseFrame /* target */, globalFrame /* source */, ros::Time(0), goalToBaseTransform);

    jointStatesSub.reset(new message_filters::Subscriber<sensor_msgs::JointState>(nh, "/joint_states", 1, ros::TransportHints(), &jointStateMessagesQueue));
    jointStatesSub->registerCallback(boost::bind(&CatchHumanController::jointStatesCallback, this, _1));

    // Spin a separate thread
    jointStateMessagesSpinner.reset(new ros::AsyncSpinner(1, &jointStateMessagesQueue));
    jointStateMessagesSpinner->start();

    // Spin until the joint states are set.
    ROS_DEBUG("Waiting for joint states to be received");
    while(true) {
        ros::Duration().fromNSec(1000000 /* 1ms */).sleep();

        // Take the read lock
        boost::shared_lock<boost::shared_mutex> lock(jointStatesAccess);
        if (jointStates.size() > 0) {
            break;
        }
    }

    ROS_DEBUG("Joint states ready");

    initMeshCache();

    readyService = nh.advertiseService("/humanoid_catching/catch_human_controller", &CatchHumanController::noop, this);

    ROS_INFO("Catch human action server initialized successfully");
}

#if ROS_VERSION_MINIMUM(1, 10, 12)
// Method not required
#else
vector<string> CatchHumanController::getActiveJointModelNames(const robot_model::JointModelGroup* jointModelGroup)
{
    vector<string> activeJointModels;
    for (unsigned int i = 0; i < jointModelGroup->getJointModels().size(); ++i)
    {
        ROS_DEBUG("Evaluating joint [%s]", jointModelGroup->getJointModels()[i]->getName().c_str());
        if (jointModelGroup->getJointModels()[i]->getMimic() != NULL)
        {
            ROS_WARN("Passive joint model found");
            continue;
        }
        activeJointModels.push_back(jointModelGroup->getJointModels()[i]->getName());

    }
    return activeJointModels;
}
#endif // ROS_VERSION_MINIMUM

CatchHumanController::~CatchHumanController()
{
}

bool CatchHumanController::noop(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    return true;
}

void CatchHumanController::publishIkCache()
{

    if (ikCachePub.getNumSubscribers() <= 0)
    {
        return;
    }

    ROS_DEBUG("Publishing the IK cache");

    IKList results;
    if (!ik->list(results))
    {
        ROS_WARN("Failed to find load ik cache for vizualization for arm [%s]", arm.c_str());
        return;
    }

    ROS_DEBUG("Publishing markers");
    unsigned int i = 0;
    visualization_msgs::MarkerArray markers;
    for (IKList::iterator j = results.begin(); j != results.end(); ++j)
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = baseFrame;
        marker.id = ++i;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
        marker.pose.position = j->point.point;
        marker.pose.orientation.w = 1.0;
        markers.markers.push_back(marker);
    }
    ikCachePub.publish(markers);

    ROS_DEBUG("Completed publishing markers");
}

void CatchHumanController::fallDetected(const std_msgs::HeaderConstPtr& fallingMsg)
{
    ROS_INFO("Human fall detected at @ %f", fallingMsg->stamp.toSec());

    // Set catching as active
    active = true;

    // Unsubscribe from further fall notifications
    humanFallSub->unsubscribe();

    if (cachedImuData.get() != NULL)
    {
        imuDataDetected(cachedImuData);
    }
}

void CatchHumanController::imuDataDetected(const sensor_msgs::ImuConstPtr& imuData)
{
    ROS_DEBUG("Human IMU data received at @ %f", imuData->header.stamp.toSec());
    cachedImuData = imuData;
    if (active)
    {
        execute(imuData);
    }
}

void CatchHumanController::reset(const std_msgs::HeaderConstPtr& reset)
{
    ROS_INFO("Resetting catching controller");

    active = false;

    // Begin listening for IMU notifications
    humanFallSub->subscribe();

    // Stop the arm
    operational_space_controllers_msgs::Move command;
    command.header.frame_id = "/torso_lift_link";
    command.header.stamp = ros::Time::now();
    command.stop = true;
    armCommandPub.publish(command);
}

void CatchHumanController::visualizeGoal(const geometry_msgs::Pose& goal, const std_msgs::Header& header,
        geometry_msgs::PoseStamped targetPose,
        geometry_msgs::TwistStamped targetVelocity,
        double humanoidRadius, double humanoidHeight) const
{
    if (goalPub.getNumSubscribers() > 0)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = header;
        pose.pose = goal;
        goalPub.publish(pose);
    }

    if (movementGoalPub.getNumSubscribers() > 0)
    {
        humanoid_catching::MovementGoal movementGoal;
        movementGoal.header = header;
        movementGoal.pose = goal;
        movementGoal.jointAngles.resize(7);

        // Get the current joint angles for the arm
        boost::shared_lock<boost::shared_mutex> lock(jointStatesAccess);
        for(unsigned int i = 0; i < 7; ++i)
        {
            const State& state = jointStates.at(jointNames.at(i));
            movementGoal.jointAngles[i] = state.position;
        }

        movementGoalPub.publish(movementGoal);
    }

    if (goalPointPub.getNumSubscribers() > 0)
    {
        geometry_msgs::PointStamped point;
        point.header = header;
        point.point = goal.position;
        goalPointPub.publish(point);
    }

    if (targetPosePub.getNumSubscribers() > 0)
    {
        targetPosePub.publish(targetPose);
    }

    if (targetVelocityPub.getNumSubscribers() > 0)
    {
        visualization_msgs::Marker arrow;
        arrow.header = targetVelocity.header;
        arrow.ns = "pole_velocity";
        arrow.id = 0;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.points.resize(2);
        arrow.points[0] = arrow.points[1] = targetPose.pose.position;
        arrow.points[1].x += targetVelocity.twist.linear.x;
        arrow.points[1].y += targetVelocity.twist.linear.y;
        arrow.points[1].z += targetVelocity.twist.linear.z;

        arrow.scale.x = 0.02;

        // arrow is yellow
        arrow.color.r = 1.0f;
        arrow.color.g = 1.0f;
        arrow.color.a = 0.5;

        targetVelocityPub.publish(arrow);
    }
}

geometry_msgs::PointStamped CatchHumanController::poseToPoint(const geometry_msgs::PoseStamped pose)
{
    geometry_msgs::PointStamped point;
    point.point = pose.pose.position;
    point.header = pose.header;
    return point;
}

double CatchHumanController::calcJointExecutionTime(const Limits& limits, const double signed_d, double v0)
{
    double d = fabs(signed_d);

    // If this is a negative distance to cover, adjust initial velocity so it is consistent.
    if (signed_d < 0)
    {
        v0 = -v0;
    }

#if (ENABLE_EXECUTION_TIME_DEBUGGING)
    ROS_DEBUG_NAMED("catch_human_action_server", "Calculating execution time for distance [%f]", signed_d);
#endif // ENABLE_EXECUTION_TIME_DEBUGGING

    // Determine the max "bang-bang" distance.
    double max_tri_t_no_v0 = 2.0 * limits.velocity / limits.acceleration;
    double max_tri_t = max_tri_t_no_v0 - v0 / limits.acceleration;
    double t0 = fabs(v0 / limits.acceleration);
    double d0 = v0 * t0 / 2.0;

    // The maximum triangular distance is either:
    // - for positive v0, the triangle minus v0 * t0, because that portion is already covered
    // - for negative v0, the triangle minus -v0 * t0, because that portion must additionally be covered
    double max_tri_distance_no_v0 = limits.velocity * max_tri_t_no_v0;
    double max_tri_distance = max_tri_distance_no_v0 - fabs(d0);

#if (ENABLE_EXECUTION_TIME_DEBUGGING)
    ROS_DEBUG_NAMED("catch_human_action_server", "Maximum triangular distance [%f], distance adjusted [%f], time [%f] given initial_vel [%f], max_vel [%f] and max_accel [%f]",
                    max_tri_distance_no_v0, max_tri_distance, max_tri_t_no_v0, v0, limits.velocity, limits.acceleration);

    ROS_DEBUG_NAMED("catch_human_action_server", "t0 [%f] d0 [%f]", t0, d0);
#endif // ENABLE_EXECUTION_TIME_DEBUGGING

    double t;

    // Required distance is less than minimum acceleration to zero time
    if (d <= d0)
    {
        t = t0;
#if (ENABLE_EXECUTION_TIME_DEBUGGING)
        ROS_DEBUG_NAMED("catch_human_action_server", "Minimum solution for t is %f", t);
#endif
    }
    else if (d <= max_tri_distance)
    {
        // Either distance is positive, in which case we add it so that we can determine
        // the triangle size and compensate for the time via t0 or distance is negative,
        // and we add the absolute value as the distance must be covered, and add the t0.
        if (d0 > 0)
        {
            double t2 = sqrt(2 * (d + d0) / limits.acceleration);
#if (ENABLE_EXECUTION_TIME_DEBUGGING)
            ROS_DEBUG_NAMED("catch_human_action_server", "t2 %f", t2);
#endif
            t = t2 - t0;
        }
        else
        {
            double t2 = sqrt(2 * (d - d0) / limits.acceleration);
#if (ENABLE_EXECUTION_TIME_DEBUGGING)
            ROS_DEBUG_NAMED("catch_human_action_server", "t2 %f", t2);
#endif
            t = t2 + t0;
        }
#if (ENABLE_EXECUTION_TIME_DEBUGGING)
        ROS_DEBUG_NAMED("catch_human_action_server", "Triangular solution for t is %f", t);
#endif
        assert (t <= max_tri_t);
    }
    else
    {
        // Remove acceleration and deacceleration distance and calculate the trapezoidal base distance
        double d_at_max_v = d - max_tri_distance;
        double t_at_max_v = d_at_max_v / limits.velocity;
        t = t_at_max_v + max_tri_t_no_v0;
        if (d0 < 0)
        {
            t += t0;
        }
#if (ENABLE_EXECUTION_TIME_DEBUGGING)
        ROS_DEBUG_NAMED("catch_human_action_server", "Trapezoidal solution for t is %f", t);
#endif
        assert(t > max_tri_t);
    }
    return t;
}

ros::Duration CatchHumanController::calcExecutionTime(const vector<double>& solution) const
{
    ROS_DEBUG("Calculating execution time for %lu joints", solution.size());
    double longestTime = 0.0;

    // Take the read lock
    boost::shared_lock<boost::shared_mutex> lock(jointStatesAccess);
    assert(solution.size() == jointNames.size());

    // TODO: Do we care about the last 2 joints here?
    for(unsigned int i = 0; i < solution.size(); ++i)
    {
        const string& jointName = jointNames.at(i);
        const Limits& limits = jointLimits.at(jointName);

        const State& state = jointStates.at(jointName);
        double v0 = state.velocity;
        double signed_d = state.position - solution[i];
#if (ENABLE_EXECUTION_TIME_DEBUGGING)
        ROS_DEBUG_NAMED("catch_human_action_server", "Distance to travel for joint %s is [%f]", jointName.c_str(), signed_d);
#endif
        double t = calcJointExecutionTime(limits, signed_d, v0);
        longestTime = max(longestTime, t);
    }

#if (ENABLE_EXECUTION_TIME_DEBUGGING)
    ROS_DEBUG_NAMED("catch_human_action_server", "Execution time is %f", longestTime);
#endif
    lock.unlock();
    return ros::Duration(longestTime);
}

const vector<FallPoint>::const_iterator CatchHumanController::findContact(const humanoid_catching::PredictFall::Response& fall) const
{
    for (vector<FallPoint>::const_iterator i = fall.points.begin(); i != fall.points.end(); ++i)
    {
        if (i->time > contactTimeTolerance)
        {
            ROS_DEBUG("Exiting search for contact at time [%f] with tolerance [%f]",
                      i->time.toSec(), contactTimeTolerance.toSec());
            break;
        }

        for (unsigned int j = 0; j < i->contacts.size(); ++j)
        {
            if (i->contacts[j].is_in_contact)
            {
                return i;
            }
        }
    }
    return fall.points.end();
}

bool CatchHumanController::linkPosition(const string& link, geometry_msgs::PoseStamped& pose, const robot_state::RobotState& currentRobotState) const
{
    robot_state::LinkState* linkState = currentRobotState.getLinkState(link);
    if (linkState == NULL)
    {
        ROS_DEBUG("Could not locate link named %s", link.c_str());
        pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = globalFrame;
        return false;
    }

    const Eigen::Affine3d& eigenPose = linkState->getGlobalLinkTransform();
    tf::poseEigenToMsg(eigenPose, pose.pose);

    ROS_DEBUG("%s position in global frame (%f %f %f)", link.c_str(), pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = globalFrame;

    return true;
}

void CatchHumanController::visualizeEEVelocity(const vector<double>& eeVelocity)
{
    if (eeVelocityVizPub.getNumSubscribers() > 0)
    {
        // Frame is the end effector of the given arm
        geometry_msgs::WrenchStamped wrench;
        wrench.header.stamp = ros::Time::now();
        wrench.header.frame_id = kinematicModel->getJointModelGroup(arm)->getLinkModelNames().back();
        wrench.wrench.force.x = eeVelocity[0];
        wrench.wrench.force.y = eeVelocity[1];
        wrench.wrench.force.z = eeVelocity[2];
        wrench.wrench.torque.x = eeVelocity[3];
        wrench.wrench.torque.y = eeVelocity[4];
        wrench.wrench.torque.z = eeVelocity[5];
        eeVelocityVizPub.publish(wrench);
    }
}

void CatchHumanController::sendTorques(const vector<double>& torques)
{
    // Execute the movement
    ROS_DEBUG("Dispatching torque command for arm %s", arm.c_str());
    operational_space_controllers_msgs::Move command;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = baseFrame;
    command.has_torques = true;

    // There may be zeros at the end of the command torque vector. Ensure we send one
    // torque per link regardless.
    command.torques.resize(7);
    for (unsigned int i = 0; i < torques.size(); ++i) {
        command.torques[i] = torques[i];
    }
    armCommandPub.publish(command);
}

void CatchHumanController::updateRavelinModel()
{
    ROS_DEBUG("Updating the Ravelin model");

    // Update the joint positions and velocities
    unsigned int numGeneralized = body->num_generalized_coordinates(Ravelin::DynamicBodyd::eEuler);

    // Copy over positions velocities
    Ravelin::VectorNd currentPositions(numGeneralized);
    currentPositions.set_zero();
    Ravelin::VectorNd currentVelocities(numGeneralized);
    currentVelocities.set_zero();

    // Lock scope
    {
        boost::shared_lock<boost::shared_mutex> lock(jointStatesAccess);
        for(map<string, int>::iterator it = jointIndices.begin(); it != jointIndices.end(); ++it)
        {
            currentPositions[it->second] = jointStates[it->first].position;
            currentVelocities[it->second] = jointStates[it->first].velocity;
        }
    }

    body->set_generalized_coordinates_euler(currentPositions);
    body->set_generalized_velocity(Ravelin::DynamicBodyd::eEuler, currentVelocities);

    ROS_DEBUG("Ravelin model updated successfully");
}

void CatchHumanController::calcArmLinks()
{
    ROS_DEBUG("Calculating arm links");

    // Use a stack to obtain a depth first search. We want to be able to label all links below the end effector
    stack<const robot_model::LinkModel*> searchLinks;
    set<const robot_model::LinkModel*> uniqueLinks;

    for (unsigned int i = 0; i < kinematicModel->getJointModelGroup(arm)->getLinkModels().size(); ++i) {
        ROS_DEBUG("Adding arm link %s", kinematicModel->getJointModelGroup(arm)->getLinkModels()[i]->getName().c_str());
        searchLinks.push(kinematicModel->getJointModelGroup(arm)->getLinkModels()[i]);
    }

    while(!searchLinks.empty())
    {
        const robot_model::LinkModel* link = searchLinks.top();
        searchLinks.pop();

        if (!uniqueLinks.insert(link).second)
        {
            continue;
        }

        allArmLinks.push_back(link);
        ROS_DEBUG("Added link %s to all arm links", link->getName().c_str());

        // Now recurse over children
        for (unsigned int j = 0; j < link->getChildJointModels().size(); ++j)
        {
            ROS_DEBUG("Adding child link %s from joint %s", link->getChildJointModels()[j]->getChildLinkModel()->getName().c_str(),
                      link->getChildJointModels()[j]->getName().c_str());

            searchLinks.push(link->getChildJointModels()[j]->getChildLinkModel());
        }
    }
}

struct ModelsMatch
{
    explicit ModelsMatch(const robot_model::LinkModel* a) : b(a) {}
    inline bool operator()(const robot_model::LinkModel* m) const
    {
        return m->getName() == b->getName();
    }
private:
    const robot_model::LinkModel* b;
};

struct ModelNamesMatch
{
    explicit ModelNamesMatch(const string& a) : b(a) {}
    inline bool operator()(const robot_model::LinkModel* m) const
    {
        return m->getName() == b;
    }
private:
    const string& b;
};


bool CatchHumanController::predictFall(const sensor_msgs::ImuConstPtr imuData, humanoid_catching::PredictFall& predictFall, ros::Duration duration)
{
    ros::WallTime start = ros::WallTime::now();

    ROS_DEBUG("Preparing to call fall prediction");
    predictFall.request.header = imuData->header;
    predictFall.request.orientation = imuData->orientation;
    predictFall.request.velocity.angular = imuData->angular_velocity;
    predictFall.request.max_time = duration;
    predictFall.request.step_size = STEP_SIZE;
    predictFall.request.result_step_size = SEARCH_RESOLUTION;
    predictFall.request.contact_time = contactTimeTolerance;

    // Lock scope
    {
        ROS_DEBUG("Sending model");
        planning_scene_monitor::LockedPlanningSceneRO planningSceneLock(planningScene);
        const vector<robot_state::LinkState*>& linkStates = planningSceneLock->getCurrentState().getLinkStateVector();

        // Reserve capacity so push_back doesn't allocate
        predictFall.request.end_effectors.reserve(allArmLinks.size());
        predictFall.request.links.reserve(linkStates.size() - allArmLinks.size());

        for (vector<robot_state::LinkState*>::const_iterator i = linkStates.begin(); i != linkStates.end(); ++i)
        {
            Link link;
            const Eigen::Affine3d& eigenPose = (*i)->getGlobalLinkTransform();

            tf::poseEigenToMsg(eigenPose, link.pose.pose);

            ROS_DEBUG("%s position in global frame (%f %f %f)", (*i)->getLinkModel()->getName().c_str(), link.pose.pose.position.x, link.pose.pose.position.y, link.pose.pose.position.z);

            link.pose.header.stamp = imuData->header.stamp;
            link.pose.header.frame_id = globalFrame;

            if ((*i)->getLinkModel()->getShape().get() == NULL) {
                ROS_DEBUG("Null shape for link [%s]", (*i)->getLinkModel()->getName().c_str());
                continue;
            }

            const shapes::Shape* shape = (*i)->getLinkModel()->getShape().get();
            if (shape->type == shapes::SPHERE) {
                link.shape.type = Shape::SPHERE;
                link.shape.dimensions.resize(1);
                link.shape.dimensions[0] = dynamic_cast<const shapes::Sphere*>(shape)->radius;
            }
            else if (shape->type == shapes::BOX) {
                link.shape.type = Shape::BOX;
                link.shape.dimensions.assign(dynamic_cast<const shapes::Box*>(shape)->size, dynamic_cast<const shapes::Box*>(shape)->size + 3);
            }
            else if (shape->type == shapes::CYLINDER) {
                link.shape.type = Shape::CYLINDER;
                const shapes::Cylinder* cylinder = dynamic_cast<const shapes::Cylinder*>(shape);
                link.shape.dimensions.resize(2);
                link.shape.dimensions[0] = cylinder->radius;
                link.shape.dimensions[1] = cylinder->length;
            }
            else if (shape->type == shapes::MESH) {
                link.shape.type = Shape::MESH;
                //const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(shape);
                //link.shape.triangles.assign(mesh->triangles, mesh->triangles + mesh->triangle_count * 3);
                // link.shape.vertices.assign(mesh->vertices, mesh->vertices + mesh->vertex_count * 3);
                const urdf::Mesh* meshResource = dynamic_cast<const urdf::Mesh*>(urdfModel->getLink((*i)->getLinkModel()->getName())->collision->geometry.get());
                link.shape.meshResource = meshResource->filename;
                link.shape.meshScale.x = meshResource->scale.x;
                link.shape.meshScale.y = meshResource->scale.y;
                link.shape.meshScale.z = meshResource->scale.z;

            }
            else {
                ROS_ERROR("Unexpected shape type: [%u]", shape->type);
            }

            link.name = (*i)->getLinkModel()->getName();

            // Decide whether to add it to end effectors or collisions
            if (find_if(allArmLinks.begin(), allArmLinks.end(), ModelsMatch((*i)->getLinkModel())) != allArmLinks.end())
            {
                predictFall.request.end_effectors.push_back(link);
                ROS_DEBUG("Adding [%s] to the end effectors list", (*i)->getLinkModel()->getName().c_str());
            }
            else
            {
                predictFall.request.links.push_back(link);
            }
        }
    }

    ROS_DEBUG("Model constructed");
    if (!fallPredictor.call(predictFall))
    {
        ROS_ERROR("Failed to call fall predictor");
        return false;
    }

    humanoid_catching::DurationStamped durationMsg;
    durationMsg.arm = arm;
    durationMsg.duration = ros::Duration((ros::WallTime::now() - start).toSec());
    durationMsg.header = imuData->header;
    fallPredictionTimePub.publish(durationMsg);
    return true;
}


bool CatchHumanController::initMeshCache()
{
    ROS_DEBUG("Preparing to call fall create mesh cache");
    CreateMeshCache request;
    request.request.header.stamp = ros::Time::now();

    // Lock scope
    {
        planning_scene_monitor::LockedPlanningSceneRO planningSceneLock(planningScene);
        const vector<robot_state::LinkState*>& linkStates = planningSceneLock->getCurrentState().getLinkStateVector();
        for (vector<robot_state::LinkState*>::const_iterator i = linkStates.begin(); i != linkStates.end(); ++i)
        {
            if ((*i)->getLinkModel()->getShape().get() == NULL) {
                ROS_DEBUG("Null shape for link [%s]", (*i)->getLinkModel()->getName().c_str());
                continue;
            }

            const shapes::Shape* rawShape = (*i)->getLinkModel()->getShape().get();
            humanoid_catching::Shape shape;
            if (rawShape->type != shapes::MESH) {
                continue;
            }

            shape.type = Shape::MESH;
            const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(rawShape);
            shape.triangles.assign(mesh->triangles, mesh->triangles + mesh->triangle_count * 3);
            shape.vertices.assign(mesh->vertices, mesh->vertices + mesh->vertex_count * 3);
            request.request.shapes.push_back(shape);
            request.request.names.push_back((*i)->getLinkModel()->getName());
        }
    }

    if (!createMeshCache.call(request))
    {
        ROS_ERROR("Failed to call create mesh cache");
        return false;
    }
    return true;
}

bool CatchHumanController::checkFeasibility(const FallPoint& fallPoint, Solution& possibleSolution, const std_msgs::Header& fallPointHeader) const
{

    // Initialize to very poor value
    possibleSolution.delta = ros::Duration(-1000);

    geometry_msgs::PoseStamped basePose;
    basePose.header = fallPointHeader;
    basePose.pose = fallPoint.pose;

    geometry_msgs::PoseStamped transformedPose = transformGoalToBase(basePose);
    possibleSolution.targetPose = transformedPose;

    possibleSolution.targetVelocity.twist.linear = transformGoalToBase(fallPoint.velocity.linear);
    possibleSolution.targetVelocity.header = transformedPose.header;

    possibleSolution.position.header = transformedPose.header;
    possibleSolution.position.point = transformedPose.pose.position;

    ros::WallTime start = ros::WallTime::now();
    bool success = false;
    possibleSolution.time = fallPoint.time;

    // Lookup the IK solution
    geometry_msgs::PointStamped point;
    point.point = transformedPose.pose.position;
    point.header = transformedPose.header;

    IKList results;
    bool queryFailed = false;
    ros::WallTime stop;
    if (!ik->query(point, results))
    {
        ROS_DEBUG("Failed to find IK solution for arm [%s]", arm.c_str());
        queryFailed = true;
        stop = ros::WallTime::now();
    }
    else {
        // We want to measure IK time, not distance estimation time
        stop = ros::WallTime::now();
        ROS_DEBUG("Received %lu results from IK query", results.size());
        for (IKList::iterator j = results.begin(); j != results.end(); ++j)
        {
            // We want to search for the fastest arm movement
            ros::Duration estimate = calcExecutionTime(j->positions);
            ros::Duration executionTimeDelta = ros::Duration(fallPoint.time) - estimate;
            if (executionTimeDelta < possibleSolution.delta)
            {
                continue;
            }
            success = true;
            possibleSolution.estimate = estimate;
            possibleSolution.delta = executionTimeDelta;
        }
    }

    if (!queryFailed && trialGoalPub.getNumSubscribers() > 0) {
        trialGoalPub.publish(poseToPoint(transformedPose));
    }

    // Other than exceptional cases, query failures indicate the target point is beyond the maximum
    // distance of the arm
    if (!queryFailed && ikMetricsPub.getNumSubscribers() > 0) {
        IKMetric msg;
        msg.arm = arm;
        msg.time = ros::Duration((stop - start).toSec());
        msg.was_successful = success;
        msg.count = results.size();
        ikMetricsPub.publish(msg);
    }
    return success;
}

const robot_model::LinkModel* CatchHumanController::findParentLinkInJointModelGroup(const robot_model::LinkModel* start) const {

    ROS_DEBUG("Searching for parent link of [%s]", start->getName().c_str());
    const robot_model::LinkModel* link = start;
    while(
          find(kinematicModel->getJointModelGroup(arm)->getLinkModels().begin(),
               kinematicModel->getJointModelGroup(arm)->getLinkModels().end(), link)
            == kinematicModel->getJointModelGroup(arm)->getLinkModels().end()) {

                const robot_model::JointModel* joint = link->getParentJointModel();
                link = joint->getParentLinkModel();
            }
            ROS_DEBUG("Parent link of link [%s] is [%s]", start->getName().c_str(), link->getName().c_str());
            return link;
}

const robot_model::JointModel* CatchHumanController::findParentActiveJoint(const robot_model::LinkModel* start) const {

    ROS_DEBUG("Searching for parent active joint model of [%s]", start->getName().c_str());
    const robot_model::JointModel* joint = start->getParentJointModel();
    while(jointIndices.find(joint->getName()) == jointIndices.end()) {
        joint = joint->getParentLinkModel()->getParentJointModel();
    }
    ROS_DEBUG("Parent joint model of link [%s] is [%s]", start->getName().c_str(), joint->getName().c_str());
    return joint;
}

unsigned int CatchHumanController::countJointsAboveInChain(const robot_model::LinkModel* contactLink) const {

    const robot_model::LinkModel* root = kinematicModel->getJointModelGroup(arm)->getLinkModels().front()->getParentJointModel()->getParentLinkModel();
    ROS_DEBUG("Searching from contact link [%s] to root link [%s]", contactLink->getName().c_str(), root->getName().c_str());

    unsigned int numJoints = 0;
    const robot_model::LinkModel* link = contactLink;

    // Count the joints
    while (link != root) {
        const robot_model::JointModel* joint = link->getParentJointModel();
        ROS_DEBUG("Parent joint [%s]", joint->getName().c_str());

        if (joint->getMimic() != NULL) {
            ROS_DEBUG("Skipping mimic joint [%s]", joint->getName().c_str());
        }
        else if (find(kinematicModel->getJointModelGroup(arm)->getJointModels().begin(),
                 kinematicModel->getJointModelGroup(arm)->getJointModels().end(), joint)
            != kinematicModel->getJointModelGroup(arm)->getJointModels().end()) {
            ROS_DEBUG("Incrementing joint count");
            ++numJoints;
        }
        else {
            ROS_DEBUG("Skipping joint not in chain [%s]", joint->getName().c_str());
        }

        link = joint->getParentLinkModel();
        ROS_DEBUG("Parent link [%s]", link->getName().c_str());
    }
    ROS_DEBUG("Found %u joints above link [%s]", numJoints, contactLink->getName().c_str());
    return numJoints;
}

void CatchHumanController::execute(const sensor_msgs::ImuConstPtr imuData)
{
    ROS_INFO("Catch procedure initiated");
    assert(imuData->header.frame_id == globalFrame);

    ros::WallTime startWallTime = ros::WallTime::now();
    ros::Time startRosTime = ros::Time::now();
    ROS_DEBUG("Planning scene is from %f", planningScene->getLastUpdateTime().toSec());

    ROS_INFO("Predicting fall for arm %s", arm.c_str());

    humanoid_catching::PredictFall predictFallObj;
    if (!predictFall(imuData, predictFallObj, MAX_DURATION))
    {
        ROS_WARN("Fall prediction failed for arm %s", arm.c_str());
        return;
    }
    ROS_INFO("Fall predicted successfully for arm %s. Predicted [%lu] steps.", arm.c_str(), predictFallObj.response.points.size());

    ROS_DEBUG("Estimated position: %f %f %f", predictFallObj.response.points[0].pose.position.x, predictFallObj.response.points[0].pose.position.y,  predictFallObj.response.points[0].pose.position.z);
    ROS_DEBUG("Estimated angular velocity: %f %f %f", predictFallObj.response.points[0].velocity.angular.x, predictFallObj.response.points[0].velocity.angular.y,  predictFallObj.response.points[0].velocity.angular.z);
    ROS_DEBUG("Estimated linear velocity: %f %f %f", predictFallObj.response.points[0].velocity.linear.x, predictFallObj.response.points[0].velocity.linear.y,  predictFallObj.response.points[0].velocity.linear.z);

    // Determine whether to execute balancing.
    const vector<FallPoint>::const_iterator fallPoint = findContact(predictFallObj.response);

    // Determine if the robot is currently in contact
    if (fallPoint != predictFallObj.response.points.end())
    {
        ROS_INFO("Robot is in contact at projected time [%f]. Executing balancing for arm %s", fallPoint->time.toSec(), arm.c_str());
        ros::Time balancingTimeStart = ros::Time::now();
        humanoid_catching::CalculateTorques calcTorques;
        calcTorques.request.name = arm;
        calcTorques.request.body_velocity = fallPoint->velocity;
        calcTorques.request.body_inertia_matrix = predictFallObj.response.inertia_matrix;
        calcTorques.request.body_mass = predictFallObj.response.body_mass;
        calcTorques.request.time_delta = ros::Duration(0.01);
        calcTorques.request.body_com = fallPoint->pose;

        // Search for the last contact in the kinematic chain
        const robot_model::LinkModel* contactLink = NULL;
        calcTorques.request.num_effective_joints = 0;
        geometry_msgs::Point contactPosition;

        for (unsigned int i = 0; i < fallPoint->contacts.size(); ++i)
        {
            if (fallPoint->contacts[i].is_in_contact)
            {
                // Locate the joint
                vector<const robot_model::LinkModel*>::const_iterator found = find_if(allArmLinks.begin(), allArmLinks.end(), ModelNamesMatch(fallPoint->contacts[i].link.name));
                assert(found != allArmLinks.end() && "Could not locate arm link in allArmLinks");
                const robot_model::LinkModel* actualContactLink = *found;

                unsigned int numEffectiveJoints = countJointsAboveInChain(actualContactLink);

                if (numEffectiveJoints > calcTorques.request.num_effective_joints) {
                    // Now determine how many joints are active above it
                    calcTorques.request.num_effective_joints = numEffectiveJoints;

                    // Find the parent link that is part of the joint model
                    contactLink = findParentLinkInJointModelGroup(actualContactLink);

                    ROS_INFO("Contact is with link [%s] (using parent link [%s]) at position: [%f %f %f] and normal: [%f %f %f]. Will use [%u] active joints.",
                            fallPoint->contacts[i].link.name.c_str(),
                            contactLink->getName().c_str(),
                            fallPoint->contacts[i].position.x,
                            fallPoint->contacts[i].position.y,
                            fallPoint->contacts[i].position.z,
                            fallPoint->contacts[i].normal.x,
                            fallPoint->contacts[i].normal.y,
                            fallPoint->contacts[i].normal.z,
                            calcTorques.request.num_effective_joints);

                    calcTorques.request.contact_normal = fallPoint->contacts[i].normal;
                    contactPosition = fallPoint->contacts[i].position;
                }
            }
        }

        assert(contactLink && "Contact not found!");

        calcTorques.request.ground_contact = predictFallObj.response.ground_contact;

        // Get the list of joints in this group
        const robot_model::JointModelGroup* jointModelGroup = kinematicModel->getJointModelGroup(arm);

        // Set the joint poses into the ravelin model
        updateRavelinModel();

        // Get the inertia matrix
        Ravelin::MatrixNd robotInertiaMatrix;
        body->get_generalized_inertia(robotInertiaMatrix);
        ROS_DEBUG_STREAM("robot intertia matrix: " << endl << robotInertiaMatrix);
        Ravelin::MatrixNd armMatrix;

        const robot_model::JointModel* parentActiveJoint = findParentActiveJoint(contactLink);
        const unsigned int startIndex = jointIndices[jointNames[0]];
        const unsigned int endIndex = jointIndices[parentActiveJoint->getName()];

        ROS_DEBUG("Extracting the intertia matrix for [%s] from [%s](%u) to [%s](%u)", contactLink->getName().c_str(), jointNames[0].c_str(),
                 startIndex, parentActiveJoint->getName().c_str(), endIndex);

        robotInertiaMatrix.get_sub_mat(startIndex, endIndex + 1,
                                       startIndex, endIndex + 1,
                                       armMatrix);

        calcTorques.request.robot_inertia_matrix = vector<double>(armMatrix.data(), armMatrix.data() + armMatrix.size());
        ROS_DEBUG("Completed extracting the inertia matrix");
        ROS_DEBUG_STREAM("robot arm intertia matrix (" << armMatrix.size() << "): " << endl << armMatrix);

        // Get the jacobian
        Eigen::MatrixXd jacobian;
        {
            // Lock scope
            planning_scene_monitor::LockedPlanningSceneRO planningSceneLock(planningScene);
            const robot_state::RobotState& currentRobotState = planningSceneLock->getCurrentState();

#if ROS_VERSION_MINIMUM(1, 10, 12)
            jacobian = currentRobotState.getJacobian(jointModelGroup, contactLink->getName());
#else
            const Eigen::Affine3d& linkCOM = currentRobotState.getLinkState(contactLink->getName())->getGlobalLinkTransform();
            ROS_DEBUG("Link COM position [%f, %f, %f] and contact position [%f, %f, %f]", linkCOM.translation()[0], linkCOM.translation()[1], linkCOM.translation()[2], contactPosition.x, contactPosition.y, contactPosition.z);
            const Eigen::Vector3d contactPositionOnLink = Eigen::Vector3d(contactPosition.x, contactPosition.y, contactPosition.z) - linkCOM.translation();
            ROS_DEBUG("Contact position on link: [%f, %f, %f]", contactPositionOnLink.x(), contactPositionOnLink.y(), contactPositionOnLink.z());
            if (!currentRobotState.getJointStateGroup(jointModelGroup->getName())->getJacobian(contactLink->getName(), contactPositionOnLink, jacobian)) {
                ROS_ERROR("Failed to  get jacobian for link [%s]", contactLink->getName().c_str());
                return;
            }
#endif
        }

        // Set current velocities
        calcTorques.request.joint_velocity.resize(calcTorques.request.num_effective_joints);
        // Lock scope
        {
            boost::shared_lock<boost::shared_mutex> lock(jointStatesAccess);
            for (unsigned int j = 0; j < calcTorques.request.num_effective_joints; ++j)
            {
                calcTorques.request.joint_velocity[j] = jointStates[jointNames[j]].velocity;
            }
        }

        // Convert to raw type
        calcTorques.request.jacobian_matrix.resize(jacobian.rows() * jacobian.cols());
        Eigen::Map<Eigen::MatrixXd>(&calcTorques.request.jacobian_matrix[0], jacobian.rows(), jacobian.cols()) = jacobian;

        // Set the joint limits
        calcTorques.request.torque_limits.resize(calcTorques.request.num_effective_joints);
        calcTorques.request.velocity_limits.resize(calcTorques.request.num_effective_joints);
        for (unsigned int j = 0; j < calcTorques.request.num_effective_joints; ++j)
        {
            double maxEffort = jointLimits[jointNames[j]].effort;
            calcTorques.request.torque_limits[j].minimum = -maxEffort;
            calcTorques.request.torque_limits[j].maximum = maxEffort;

            double maxVelocity = jointLimits[jointNames[j]].velocity;
            calcTorques.request.velocity_limits[j].minimum = -maxVelocity;
            calcTorques.request.velocity_limits[j].maximum = maxVelocity;
        }

        ROS_DEBUG("Calculating torques");
        if (!balancer.call(calcTorques))
        {
            ROS_WARN("Calculating torques failed");
            return;
        }
        ROS_DEBUG("Torques calculated successfully");

        // Visualize the desired velocity
        visualizeEEVelocity(calcTorques.response.ee_velocities);
        ostringstream os;
        for (unsigned int i = 0; i < calcTorques.response.torques.size(); ++i) {
            os << calcTorques.response.torques[i];
            if (i !=  calcTorques.response.torques.size() - 1) {
                os << ", ";
            }
        }
        ROS_INFO("Torques [%s]", os.str().c_str());
        humanoid_catching::DurationStamped durationMsg;
        durationMsg.arm = arm;
        durationMsg.duration = ros::Duration((ros::Time::now() - balancingTimeStart).toSec());
        durationMsg.header = imuData->header;
        balancingTimePub.publish(durationMsg);
        sendTorques(calcTorques.response.torques);
    }
    else
    {
        ROS_INFO("Robot is not in contact. Beginning search for intercept solutions");
        ros::Time start = ros::Time::now();
        ros::WallTime wallStart = ros::WallTime::now();

        vector<Solution> solutions;
        // We now have a projected time/position path. Search the path for acceptable times.
        // Search for a fixed duration of time
        unsigned int level = 0;
        while (ros::Time::now() - start < MAX_GRID_SEARCH_DURATION) {

            unsigned int quadrants = pow(2, level);

            for (unsigned int q = 0; q < quadrants && ros::Time::now() - start < MAX_GRID_SEARCH_DURATION; ++q) {

                // Determine the size of each quadrant
                unsigned int pointQuadrantSize = min(ceil(1 / double(quadrants) * predictFallObj.response.points.size()), double(predictFallObj.response.points.size()));

                // Find the beginning of the quadrant and then take the midpoint
                unsigned int point = min(q * pointQuadrantSize + int(floor(pointQuadrantSize / 2)), (unsigned int)predictFallObj.response.points.size() - 1);

                // Determine the size of each quadrant
                double heightQuadrantSize = 1 / double(quadrants) * predictFallObj.response.height;

                // Find the beginning of the quadrant and then take the midpoint
                double height = q * heightQuadrantSize + heightQuadrantSize / 2.0;

                geometry_msgs::Pose comPose = predictFallObj.response.points[point].pose;
                tf::Quaternion rotation(comPose.orientation.x, comPose.orientation.y, comPose.orientation.z, comPose.orientation.w);

                tf::Vector3 initialVector(1, 0, 0);
                tf::Vector3 rotatedVector = tf::quatRotate(rotation, initialVector);

                // Now set the height. Height is assumed to be between [0, height], so compensate for the COM at the center of the pole.
                rotatedVector *= (height - predictFallObj.response.height / 2.0);

                // Now offset by the COM
                rotatedVector += tf::Vector3(comPose.position.x, comPose.position.y, comPose.position.z);

                FallPoint fallPointQuery = predictFallObj.response.points[point];
                fallPointQuery.pose.position.x = rotatedVector.x();
                fallPointQuery.pose.position.y = rotatedVector.y();
                fallPointQuery.pose.position.z = rotatedVector.z();

                Solution possible;
                ROS_DEBUG("Checking for feasibility at position [%f, %f, %f] given COM [%f, %f, %f] and height %f",
                fallPointQuery.pose.position.x, fallPointQuery.pose.position.y, fallPointQuery.pose.position.z, comPose.position.x, comPose.position.y, comPose.position.z, height);
                if (checkFeasibility(fallPointQuery, possible, predictFallObj.response.header))
                {
                    possible.height = height;
                    solutions.push_back(possible);
                }
            }
            // Increment the number of quadrants
            ++level;
        }

        ROS_INFO("Ending search for solutions. Searched [%u] levels and found [%lu] possible solutions", level, solutions.size());

        // Iterate over all possible solutions and select the most feasible. Use a progressive relaxation of the time
        // constraint.
        const Solution* bestSolution = NULL;

        if (solutions.empty())
        {
            ROS_INFO("Possible solution set is empty. Aborting search.");
        }
        else {
            ros::Duration durationLimit = ros::Duration(0);
            while (!bestSolution) {
                for (vector<Solution>::const_iterator possible = solutions.begin(); possible != solutions.end(); ++possible) {
                    if (possible->delta >= durationLimit && (!bestSolution || bestSolution->height < possible->height)) {
                        bestSolution = &*possible;
                    }
                }
                if (!bestSolution) {
                    // TODO: Assess if this value should be lower
                    durationLimit -= ros::Duration(0.1);
                }
            }
            ROS_INFO("Relaxed constraint to [%f] to find solution", durationLimit.toSec());
        }

        if (!bestSolution)
        {
            ROS_WARN("No possible catch positions for arm [%s]. Sending zero torques", arm.c_str());
            vector<double> zeroTorques;
            zeroTorques.resize(7);
            sendTorques(zeroTorques);
        }
        else {
            ROS_INFO("Publishing command for arm %s. Solution selected based on target pose with position (%f %f %f) and orientation (%f %f %f %f) @ time [%f] with delta [%f] and estimate [%f] and height [%f]",
                    arm.c_str(), bestSolution->targetPose.pose.position.x, bestSolution->targetPose.pose.position.y, bestSolution->targetPose.pose.position.z,
                    bestSolution->targetPose.pose.orientation.x, bestSolution->targetPose.pose.orientation.y, bestSolution->targetPose.pose.orientation.z, bestSolution->targetPose.pose.orientation.w,
                    bestSolution->time.toSec(), bestSolution->delta.toSec(), bestSolution->estimate.toSec(), bestSolution->height);

            operational_space_controllers_msgs::Move command;

            ROS_DEBUG("Calculating ee_pose for link %s", kinematicModel->getJointModelGroup(arm)->getLinkModelNames().back().c_str());
            geometry_msgs::PoseStamped eePose;

            {
                // lock scope
                planning_scene_monitor::LockedPlanningSceneRO planningSceneLock(planningScene);
                if (!linkPosition(kinematicModel->getJointModelGroup(arm)->getLinkModelNames().back(), eePose, planningSceneLock->getCurrentState()))
                {
                    ROS_WARN("Failed to find ee pose. Using default.");
                }
            }

            // Convert to baseFrame
            eePose = transformGoalToBase(eePose);

            command.header = bestSolution->position.header;
            command.target.position = bestSolution->position.point;
            command.target.orientation = computeOrientation(*bestSolution, eePose);
            command.point_at_target = false;
            visualizeGoal(command.target, command.header, bestSolution->targetPose, bestSolution->targetVelocity, predictFallObj.response.radius,
                        predictFallObj.response.height);
            armCommandPub.publish(command);
        }

        humanoid_catching::DurationStamped durationMsg;
        durationMsg.arm = arm;
        durationMsg.duration = ros::Duration((ros::Time::now() - start).toSec());
        durationMsg.header = imuData->header;
        interceptTimePub.publish(durationMsg);
    }

    ROS_INFO("Reaction time was %f(s) wall time and %f(s) clock time", ros::WallTime::now().toSec() - startWallTime.toSec(),
             ros::Time::now().toSec() - startRosTime.toSec());
    humanoid_catching::DurationStamped durationMsg;
    durationMsg.arm = arm;
    durationMsg.duration = ros::Duration(ros::Time::now().toSec() - startRosTime.toSec());
    durationMsg.header = imuData->header;
    reactionTimePub.publish(durationMsg);
}

/* static */ geometry_msgs::Quaternion CatchHumanController::computeOrientation(const Solution& solution, const geometry_msgs::PoseStamped& currentPose)
{
    ROS_DEBUG("q_pole: %f %f %f %f", solution.targetPose.pose.orientation.x,
              solution.targetPose.pose.orientation.y,
              solution.targetPose.pose.orientation.z,
              solution.targetPose.pose.orientation.w);

    tf::Quaternion qPole;
    tf::quaternionMsgToTF(solution.targetPose.pose.orientation, qPole);

    ROS_DEBUG("q_hand: %f %f %f %f", currentPose.pose.orientation.x,
              currentPose.pose.orientation.y,
              currentPose.pose.orientation.z,
              currentPose.pose.orientation.w);

    tf::Quaternion qHand;
    tf::quaternionMsgToTF(currentPose.pose.orientation, qHand);

    ROS_DEBUG("v_pole: %f %f %f",
              solution.targetVelocity.twist.linear.x,
              solution.targetVelocity.twist.linear.y,
              solution.targetVelocity.twist.linear.z);

    // Create a quaternion representing the linear velocity
    tf::Vector3 l(solution.targetVelocity.twist.linear.x, solution.targetVelocity.twist.linear.y, solution.targetVelocity.twist.linear.z);
    if (l.length() == 0)
    {
        l.setX(tfScalar(1));
    }
    l.normalize();
    tf::Quaternion qL(quaternionFromVector(l));

    ROS_DEBUG("pose_target: %f %f %f",
              solution.targetPose.pose.position.x,
              solution.targetPose.pose.position.y,
              solution.targetPose.pose.position.z);

    ROS_DEBUG("pose_current: %f %f %f",
              currentPose.pose.position.x,
              currentPose.pose.position.y,
              currentPose.pose.position.z);
    // Rotate to a vector orthoganal to the velocity.

    // Create the vector representing the direction from the end effector to pole
    // COM
    tf::Vector3 pointAt(solution.targetPose.pose.position.x - currentPose.pose.position.x,
                        solution.targetPose.pose.position.y - currentPose.pose.position.y,
                        solution.targetPose.pose.position.z - currentPose.pose.position.z);

    ROS_DEBUG("p_at: %f %f %f", pointAt.x(), pointAt.y(), pointAt.z());

    pointAt.normalize();

    ROS_DEBUG("p_at: %f %f %f", pointAt.x(), pointAt.y(), pointAt.z());

    tf::Quaternion qYaw;
    qYaw.setRPY(0, 0, pi / 2);

    tf::Quaternion qYawNegative;
    qYawNegative.setRPY(0, 0, -pi / 2);

    tf::Vector3 pos = quatToVector(qL * qYaw);
    ROS_DEBUG("Positive rotation %f %f %f", pos.x(), pos.y(), pos.z());

    tf::Vector3 neg = quatToVector(qL * qYawNegative);
    ROS_DEBUG("Negative rotation %f %f %f", neg.x(), neg.y(), neg.z());

    ROS_DEBUG("Distance positive %f Distance negative %f", pointAt.distance2(quatToVector(qL * qYaw)), pointAt.distance2(quatToVector(qL * qYawNegative)));
    if (pointAt.distance2(quatToVector(qL * qYaw)) <= pointAt.distance2(quatToVector(qL * qYawNegative)))
    {
        qL *= qYaw;
    }
    else
    {
        qL *= qYawNegative;
    }

    // Rotate so larger contact surface contacts pole. Determine the shortest rotation.
    tf::Quaternion qRoll;
    qRoll.setRPY(pi / 2, 0, 0);

    tf::Quaternion qRollNegative;
    qRollNegative.setRPY(-pi / 2, 0, 0);

    // TODO: QL is wrong base?
    if (qHand.angleShortestPath(qL * qRoll) <= qHand.angleShortestPath(qL * qRollNegative))
    {
        qL *= qRoll;
    }
    else
    {
        qL *= qRollNegative;
    }
    qL.normalize();

    geometry_msgs::Quaternion orientation;
    tf::quaternionTFToMsg(qL, orientation);
    return orientation;
}

#if !defined(ENABLE_TESTING)
int main(int argc, char** argv)
{
    ros::init(argc, argv, "catch_human_action");

    CatchHumanController cha;
    ros::spin();
}
#endif
