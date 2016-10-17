#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <humanoid_catching/CatchHumanAction.h>
#include <kinematics_cache/IKQueryv2.h>
#include <humanoid_catching/PredictFall.h>
#include <humanoid_catching/catch_human_action_server.h>
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
#include <ros/spinner.h>

using namespace std;
using namespace humanoid_catching;

static const double MAX_VELOCITY = 100;
static const double MAX_ACCELERATION = 100;
static const double MAX_EFFORT = 100;

//! Tolerance of time to be considered in contact
static const ros::Duration CONTACT_TIME_TOLERANCE = ros::Duration(0.1);
static const ros::Duration MAX_DURATION = ros::Duration(10.0);
static const ros::Duration STEP_SIZE = ros::Duration(0.001);

static const ros::Duration SEARCH_RESOLUTION(0.05);
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

geometry_msgs::Pose CatchHumanActionServer::applyTransform(const geometry_msgs::PoseStamped& pose, const tf::StampedTransform transform)
{
    tf::Stamped<tf::Pose> tfPose;
    tf::poseStampedMsgToTF(pose, tfPose);
    tf::Pose tfGoal = transform * tfPose;
    geometry_msgs::Pose msgGoal;
    tf::poseTFToMsg(tfGoal, msgGoal);
    return msgGoal;
}

geometry_msgs::Vector3 CatchHumanActionServer::applyTransform(const geometry_msgs::Vector3& linear, const tf::StampedTransform transform)
{
    tf::Stamped<tf::Vector3> tfLinear;

    tf::vector3MsgToTF(linear, tfLinear);
    tf::Vector3 tfGoal = transform * tfLinear;

    geometry_msgs::Vector3 msgGoal;
    tf::vector3TFToMsg(tfGoal, msgGoal);
    return msgGoal;
}

void CatchHumanActionServer::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
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

CatchHumanActionServer::CatchHumanActionServer(const string& name) :
    pnh("~"),
    as(nh, name, boost::bind(&CatchHumanActionServer::execute, this, _1), false),
    planningScene(new planning_scene_monitor::PlanningSceneMonitor("robot_description"))
{

    ROS_INFO("Initializing the catch human action");

    // Configure the planning scene
    planningScene->startStateMonitor();
    planningScene->startSceneMonitor();
    planningScene->startWorldGeometryMonitor();

    // Configure the fall prediction service
    ROS_INFO("Waiting for predict_fall service");
    ros::service::waitForService("/fall_predictor/predict_fall");
    fallPredictor = nh.serviceClient<humanoid_catching::PredictFall>("/fall_predictor/predict_fall", true /* persistent */);

    double maxDistance;
    pnh.param("max_distance", maxDistance, 0.821000);

    pnh.param<string>("base_frame", baseFrame, "/torso_lift_link");

    if (!pnh.getParam("arm", arm)) {
        ROS_ERROR("Arm name must be specified");
    }

    if (!pnh.getParam("ee_frame", eeFrame)) {
        ROS_ERROR("ee_frame must be specified");
    }

    string armCommandTopic;
    if (!pnh.getParam("command_topic", armCommandTopic)) {
        ROS_ERROR("command_topic must be specified");
    }

    string cacheDataName;
    if (!pnh.getParam("cache_data_name", cacheDataName)) {
        ROS_ERROR("cache_data_name must be specified");
    }

    ik.reset(new kinematics_cache::KinematicsCache(maxDistance, baseFrame, cacheDataName));

    ROS_INFO("Waiting for /balancer/torques service");
    ros::service::waitForService("/balancer/torques");
    balancer = nh.serviceClient<humanoid_catching::CalculateTorques>("/balancer/torques", true /* persistent */);

    // Initialize arm clients
    ROS_INFO("Initializing arm command publisher");
    armCommandPub = nh.advertise<operational_space_controllers_msgs::Move>(armCommandTopic, 1, false);

    // Initialize visualization publishers
    goalPub = pnh.advertise<geometry_msgs::PoseStamped>("movement_goal", 10, true);
    targetPosePub = pnh.advertise<geometry_msgs::PoseStamped>("target_pose", 1);
    targetVelocityPub = pnh.advertise<visualization_msgs::Marker>("target_velocity", 1);
    trialGoalPub = pnh.advertise<geometry_msgs::PointStamped>("movement_goal_trials", 1);
    eeVelocityVizPub = pnh.advertise<geometry_msgs::WrenchStamped>("ee_velocity", 1);

    rdf_loader::RDFLoader rdfLoader;
    const boost::shared_ptr<srdf::Model> &srdf = rdfLoader.getSRDF();
    const boost::shared_ptr<urdf::ModelInterface>& urdfModel = rdfLoader.getURDF();
    kinematicModel.reset(new robot_model::RobotModel(urdfModel, srdf));
    ROS_INFO("Robot model initialized successfully");

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
    body->set_links_and_joints(links, joints);
    body->set_floating_base(false);

    const robot_model::JointModelGroup* jointModelGroup =  kinematicModel->getJointModelGroup(arm);

    // Determine the indexes for the arms within the body joints in Ravelin
    int curr = 0;
    for (int l = 0; l < joints.size(); ++l)
    {
        // Only record joints that represent generalized coordinates
        if (joints[l]->num_dof() > 0)
        {
            jointIndices[joints[l]->joint_id] = curr;
        }
        curr += joints[l]->num_dof();
    }

    ROS_DEBUG("Loading joint limits for arm %s", arm.c_str());

#if ROS_VERSION_MINIMUM(1, 10, 12)
    const vector<string> jointModelNames = jointModelGroup->getActiveJointModelNames();
#else
    const vector<string> jointModelNames = getActiveJointModelNames(jointModelGroup);
#endif

    for (unsigned int i = 0; i < jointModelNames.size(); ++i)
    {
        jointNames.push_back(jointModelNames[i]);

        const string prefix = "robot_description_planning/joint_limits/" + jointModelNames[i] + "/";
        ROS_DEBUG_NAMED("catch_human_action_server", "Loading velocity and acceleration limits for joint %s", jointModelNames[i].c_str());

        bool has_vel_limits;
        double max_velocity;
        if (nh.getParam(prefix + "has_velocity_limits", has_vel_limits) && has_vel_limits && nh.getParam(prefix + "max_velocity", max_velocity))
        {
            ROS_DEBUG_NAMED("catch_human_action_server", "Setting max velocity to %f", max_velocity);
        }
        else
        {
            ROS_DEBUG_NAMED("catch_human_action_server", "Setting max velocity to default");
            max_velocity = MAX_VELOCITY;
        }

        bool has_acc_limits;
        double max_acc;
        if (nh.getParam(prefix + "has_acceleration_limits", has_acc_limits) && has_acc_limits && nh.getParam(prefix + "max_acceleration", max_acc))
        {
            ROS_DEBUG_NAMED("catch_human_action_server", "Setting max acceleration to %f", max_acc);
        }
        else
        {
            ROS_DEBUG_NAMED("catch_human_action_server", "Setting max acceleration to default");
            max_acc = MAX_ACCELERATION;
        }

        // Fetch the effort from the urdf
        double max_effort;
        const urdf::Joint* ujoint = urdfModel->getJoint(jointModelNames[i]).get();
        if (ujoint != NULL && ujoint->limits)
        {
            max_effort = ujoint->limits->effort;
            ROS_DEBUG_NAMED("catch_human_action_server", "Setting max effort to %f for %s", max_effort, jointModelNames[i].c_str());
        }
        else
        {
            ROS_DEBUG_NAMED("catch_human_action_server", "Setting max effort to default for %s", jointModelNames[i].c_str());
            max_effort = MAX_EFFORT;
        }
        jointLimits[jointModelNames[i]] = Limits(max_velocity, max_acc, max_effort);
    }

    ROS_DEBUG("Completed initializing the joint limits");

    jointStatesSub.reset(new message_filters::Subscriber<sensor_msgs::JointState>(nh, "/joint_states", 1, ros::TransportHints(), &jointStateMessagesQueue));
    jointStatesSub->registerCallback(boost::bind(&CatchHumanActionServer::jointStatesCallback, this, _1));

    // Spin a separate thread
    jointStateMessagesSpinner.reset(new ros::AsyncSpinner(1, &jointStateMessagesQueue));
    jointStateMessagesSpinner->start();

    ROS_DEBUG("Starting the action server");
    as.registerPreemptCallback(boost::bind(&CatchHumanActionServer::preempt, this));
    as.start();
    ROS_INFO("Catch human action server initialized successfully");
}

#if ROS_VERSION_MINIMUM(1, 10, 12)
// Method not required
#else
vector<string> CatchHumanActionServer::getActiveJointModelNames(const robot_model::JointModelGroup* jointModelGroup)
{
    vector<string> activeJointModels;
    for (unsigned int i = 0; i < jointModelGroup->getJointModels().size(); ++i)
    {
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

void CatchHumanActionServer::preempt()
{
    ROS_WARN("Preempt request received");
    // Preemption is not supported
}

void CatchHumanActionServer::visualizeGoal(const geometry_msgs::Pose& goal, const std_msgs::Header& header,
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

geometry_msgs::PointStamped CatchHumanActionServer::poseToPoint(const geometry_msgs::PoseStamped pose)
{
    geometry_msgs::PointStamped point;
    point.point = pose.pose.position;
    point.header = pose.header;
    return point;
}

double CatchHumanActionServer::calcJointExecutionTime(const Limits& limits, const double signed_d, double v0)
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

ros::Duration CatchHumanActionServer::calcExecutionTime(const vector<double>& solution)
{
    double longestTime = 0.0;
    for(unsigned int i = 0; i < solution.size(); ++i)
    {
        const string& jointName = jointNames[i];
        Limits& limits = jointLimits[jointName];

        // Take the read lock
        boost::shared_lock<boost::shared_mutex> lock(jointStatesAccess);
        double v0 = jointStates[jointName].velocity;
        double signed_d = jointStates[jointName].position - solution[i];
        lock.unlock();

#if (ENABLE_EXECUTION_TIME_DEBUGGING)
        ROS_DEBUG_NAMED("catch_human_action_server", "Distance to travel for joint %s is [%f]", jointName.c_str(), signed_d);
#endif
        double t = calcJointExecutionTime(limits, signed_d, v0);
        longestTime = max(longestTime, t);
    }

#if (ENABLE_EXECUTION_TIME_DEBUGGING)
    ROS_DEBUG_NAMED("catch_human_action_server", "Execution time is %f", longestTime);
#endif
    return ros::Duration(longestTime);
}

geometry_msgs::Pose CatchHumanActionServer::tfFrameToPose(const string& tfFrame, const ros::Time& stamp, const string& base) const
{
    tf::StampedTransform tfStampedTransform;
    tf.lookupTransform(base, tfFrame, stamp, tfStampedTransform);
    geometry_msgs::TransformStamped stampedTransform;
    transformStampedTFToMsg(tfStampedTransform, stampedTransform);
    geometry_msgs::Pose pose;
    pose.position.x = stampedTransform.transform.translation.x;
    pose.position.y = stampedTransform.transform.translation.y;
    pose.position.z = stampedTransform.transform.translation.z;
    pose.orientation = stampedTransform.transform.rotation;
    return pose;
}

const vector<FallPoint>::const_iterator CatchHumanActionServer::findContact(const humanoid_catching::PredictFall::Response& fall) const
{
    for (vector<FallPoint>::const_iterator i = fall.points.begin(); i != fall.points.end(); ++i)
    {
        if (i->contacts[0].is_in_contact)
        {
            return i;
        }
    }
    return fall.points.end();
}

geometry_msgs::Pose CatchHumanActionServer::endEffectorPosition(const string& frame) const
{
    return tfFrameToPose(eeFrame, ros::Time(0), frame);
}

void CatchHumanActionServer::visualizeEEVelocity(const vector<double>& eeVelocity)
{
    if (eeVelocityVizPub.getNumSubscribers() > 0)
    {
        // Frame is the end effector of the given arm
        geometry_msgs::WrenchStamped wrench;
        wrench.header.stamp = ros::Time::now();
        wrench.header.frame_id = eeFrame;
        wrench.wrench.force.x = eeVelocity[0];
        wrench.wrench.force.y = eeVelocity[1];
        wrench.wrench.force.z = eeVelocity[2];
        wrench.wrench.torque.x = eeVelocity[3];
        wrench.wrench.torque.y = eeVelocity[4];
        wrench.wrench.torque.z = eeVelocity[5];
        eeVelocityVizPub.publish(wrench);
    }
}

void CatchHumanActionServer::sendTorques(const vector<double>& torques)
{
    // Execute the movement
    ROS_DEBUG("Dispatching torque command for arm %s", arm.c_str());
    operational_space_controllers_msgs::Move command;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = baseFrame;
    command.has_torques = true;
    command.torques = torques;
    armCommandPub.publish(command);
}

void CatchHumanActionServer::updateRavelinModel()
{
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
}

bool CatchHumanActionServer::predictFall(const humanoid_catching::CatchHumanGoalConstPtr& human, humanoid_catching::PredictFall& predictFall, ros::Duration duration, bool includeEndEffectors, bool visualize)
{
    predictFall.request.header = human->header;
    predictFall.request.orientation = human->orientation;
    predictFall.request.velocity = human->velocity;
    predictFall.request.accel = human->accel;
    predictFall.request.max_time = duration;
    predictFall.request.step_size = STEP_SIZE;
    predictFall.request.result_step_size = SEARCH_RESOLUTION;
    predictFall.request.visualize = visualize;
    if (includeEndEffectors)
    {
        predictFall.request.end_effector_velocities.resize(1);

        predictFall.request.end_effectors.resize(1);
        predictFall.request.end_effectors[0] = endEffectorPosition(human->header.frame_id);
    }

    if (!fallPredictor.call(predictFall))
    {
        return false;
    }
    return true;
}

void CatchHumanActionServer::execute(const humanoid_catching::CatchHumanGoalConstPtr& goal)
{

    ROS_DEBUG("Catch procedure initiated");

    if(!as.isActive() || as.isPreemptRequested() || !ros::ok())
    {
        ROS_INFO("Catch human action canceled before started");
        as.setPreempted();
        return;
    }

    ros::WallTime startWallTime = ros::WallTime::now();
    ros::Time startRosTime = ros::Time::now();

    ROS_DEBUG("Predicting fall");
    humanoid_catching::PredictFall predictFallObj;
    if (!predictFall(goal, predictFallObj, CONTACT_TIME_TOLERANCE, true, false))
    {
        ROS_INFO("Fall prediction failed");
        as.setAborted();
        return;
    }
    ROS_DEBUG("Fall predicted successfully");

    // Repredict without any end effectors
    ROS_DEBUG("Predicting fall without contact");
    humanoid_catching::PredictFall predictFallNoEE;
    if (!predictFall(goal, predictFallNoEE, MAX_DURATION, false, true))
    {
        ROS_DEBUG("Fall prediction failed");
        as.setAborted();
        return;
    }
    ROS_DEBUG("Fall predicted without contact successfully");

    ROS_DEBUG("Estimated position: %f %f %f", predictFallObj.response.points[0].pose.position.x, predictFallObj.response.points[0].pose.position.y,  predictFallObj.response.points[0].pose.position.z);
    ROS_DEBUG("Estimated angular velocity: %f %f %f", predictFallObj.response.points[0].velocity.angular.x, predictFallObj.response.points[0].velocity.angular.y,  predictFallObj.response.points[0].velocity.angular.z);
    ROS_DEBUG("Estimated linear velocity: %f %f %f", predictFallObj.response.points[0].velocity.linear.x, predictFallObj.response.points[0].velocity.linear.y,  predictFallObj.response.points[0].velocity.linear.z);

    geometry_msgs::Pose eePose = endEffectorPosition(baseFrame);

    // Determine whether to execute balancing.
    const vector<FallPoint>::const_iterator fallPoint = findContact(predictFallObj.response);

    // Determine if the robot is currently in contact
    if (fallPoint != predictFallObj.response.points.end())
    {
        ROS_INFO("Robot is in contact. Executing balancing for arm %s", arm.c_str());

        updateRavelinModel();

        humanoid_catching::CalculateTorques calcTorques;
        calcTorques.request.name = arm;
        calcTorques.request.body_velocity = fallPoint->velocity;
        calcTorques.request.body_inertia_matrix = predictFallObj.response.inertia_matrix;
        calcTorques.request.body_mass = predictFallObj.response.body_mass;
        calcTorques.request.time_delta = ros::Duration(0.01);
        calcTorques.request.body_com = fallPoint->pose;
        calcTorques.request.contact_position = fallPoint->contacts[0].position;
        calcTorques.request.contact_normal = fallPoint->contacts[0].normal;
        calcTorques.request.ground_contact = fallPoint->ground_contact;

        // Get the list of joints in this group
        const robot_model::JointModelGroup* jointModelGroup = kinematicModel->getJointModelGroup(arm);

        // Get the inertia matrix
        Ravelin::MatrixNd robotInertiaMatrix;
        body->get_generalized_inertia(robotInertiaMatrix);
        Ravelin::MatrixNd armMatrix;
        robotInertiaMatrix.get_sub_mat(jointIndices[jointNames[0]], jointIndices[jointNames.back()] + 1,
                                       jointIndices[jointNames[0]], jointIndices[jointNames.back()] + 1, armMatrix);

        calcTorques.request.robot_inertia_matrix = vector<double>(armMatrix.data(), armMatrix.data() + armMatrix.size());

        // Get the jacobian
        robot_state::RobotState currentRobotState = planningScene->getPlanningScene()->getCurrentState();

#if ROS_VERSION_MINIMUM(1, 10, 12)
        Eigen::MatrixXd jacobian = currentRobotState.getJacobian(jointModelGroup);
#else
        Eigen::MatrixXd jacobian;
        currentRobotState.getJointStateGroup(jointModelGroup->getName())->getJacobian(jointModelGroup->getLinkModelNames().back(), Eigen::Vector3d(0, 0, 0), jacobian);
#endif

        // Set current velocities
        calcTorques.request.joint_velocity.resize(jointNames.size());
        // Lock scope
        {
            boost::shared_lock<boost::shared_mutex> lock(jointStatesAccess);
            for (unsigned int j = 0; j < jointNames.size(); ++j)
            {
                calcTorques.request.joint_velocity[j] = jointStates[jointNames[j]].velocity;
            }
        }

        // Convert to raw type
        calcTorques.request.jacobian_matrix.resize(jacobian.rows() * jacobian.cols());
        Eigen::Map<Eigen::MatrixXd>(&calcTorques.request.jacobian_matrix[0], jacobian.rows(), jacobian.cols()) = jacobian;

        // Set the joint limits
        calcTorques.request.torque_limits.resize(jointNames.size());
        calcTorques.request.velocity_limits.resize(jointNames.size());
        for (unsigned int j = 0; j < jointNames.size(); ++j)
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
            as.setAborted();
            return;
        }
        ROS_DEBUG("Torques calculated successfully");

        // Visualize the desired velocity
        visualizeEEVelocity(calcTorques.response.ee_velocities);
        sendTorques(calcTorques.response.torques);
    }
    else
    {
        tf::StampedTransform goalToTorsoTransform;
        tf.lookupTransform(baseFrame, predictFallNoEE.response.header.frame_id, ros::Time(0), goalToTorsoTransform);

        // We now have a projected time/position path. Search the path for acceptable times.
        boost::optional<Solution> bestSolution;

        for (vector<FallPoint>::const_iterator i = predictFallNoEE.response.points.begin(); i != predictFallNoEE.response.points.end(); ++i)
        {
            Solution possibleSolution;

            // Initialize to a large negative number.
            possibleSolution.delta = ros::Duration(-1000);
            possibleSolution.time = i->time;

            geometry_msgs::PoseStamped basePose;
            basePose.header = predictFallNoEE.response.header;
            basePose.pose = i->pose;

            geometry_msgs::PoseStamped transformedPose;
            transformedPose.header.frame_id = baseFrame;
            transformedPose.header.stamp = predictFallNoEE.response.header.stamp;

            transformedPose.pose = applyTransform(basePose, goalToTorsoTransform);
            possibleSolution.targetPose = transformedPose;

            possibleSolution.targetVelocity.twist.linear = applyTransform(i->velocity.linear, goalToTorsoTransform);
            possibleSolution.targetVelocity.header = transformedPose.header;

            possibleSolution.position.header = transformedPose.header;
            possibleSolution.position.point = transformedPose.pose.position;

            if (trialGoalPub.getNumSubscribers() > 0)
            {
                trialGoalPub.publish(poseToPoint(transformedPose));
            }

            // Lookup the IK solution
            geometry_msgs::PointStamped point;
            point.point = transformedPose.pose.position;
            point.header = transformedPose.header;

            IKList results;
            if (!ik->query(point, results))
            {
                ROS_DEBUG("Failed to find IK solution for arm [%s]", arm.c_str());
                continue;
            }

            ROS_DEBUG("Received %lu results from IK query", results.size());
            for (IKList::iterator j = results.begin(); j != results.end(); ++j)
            {
                // We want to search for the fastest arm movement. The idea is that the first arm there should slow the human
                // down and give the second arm time to arrive.
                ros::Duration executionTimeDelta = ros::Duration(i->time) - calcExecutionTime(j->positions);
                if (bestSolution && executionTimeDelta < bestSolution->delta)
                {
                    continue;
                }

                possibleSolution.delta = executionTimeDelta;
                bestSolution = possibleSolution;
            }
        }

        if (!bestSolution)
        {
            ROS_WARN("No possible catch positions for arm [%s]", arm.c_str());
            as.setAborted();
            return;
        }

        ROS_INFO("Publishing command for arm %s. Solution selected based on target pose with position (%f %f %f) and orientation (%f %f %f %f)  @ time %f",
                 arm.c_str(), bestSolution->targetPose.pose.position.x, bestSolution->targetPose.pose.position.y, bestSolution->targetPose.pose.position.z,
                 bestSolution->targetPose.pose.orientation.x, bestSolution->targetPose.pose.orientation.y, bestSolution->targetPose.pose.orientation.z, bestSolution->targetPose.pose.orientation.w,
                 bestSolution->time.toSec());

        operational_space_controllers_msgs::Move command;

        command.header = bestSolution->position.header;
        command.target.position = bestSolution->position.point;
        command.target.orientation = computeOrientation(*bestSolution, eePose);
        command.point_at_target = false;
        visualizeGoal(command.target, command.header, bestSolution->targetPose, bestSolution->targetVelocity, predictFallNoEE.response.radius,
                      predictFallNoEE.response.height);
        armCommandPub.publish(command);
    }

    ROS_INFO("Reaction time was %f(s) wall time and %f(s) clock time", ros::WallTime::now().toSec() - startWallTime.toSec(),
             ros::Time::now().toSec() - startRosTime.toSec());
    as.setSucceeded(result);
}

/* static */ geometry_msgs::Quaternion CatchHumanActionServer::computeOrientation(const Solution& solution, const geometry_msgs::Pose& currentPose)
{

    ROS_DEBUG("q_pole: %f %f %f %f", solution.targetPose.pose.orientation.x,
             solution.targetPose.pose.orientation.y,
             solution.targetPose.pose.orientation.z,
             solution.targetPose.pose.orientation.w);

    tf::Quaternion qPole;
    tf::quaternionMsgToTF(solution.targetPose.pose.orientation, qPole);

    ROS_DEBUG("q_hand: %f %f %f %f", currentPose.orientation.x,
             currentPose.orientation.y,
             currentPose.orientation.z,
             currentPose.orientation.w);

    tf::Quaternion qHand;
    tf::quaternionMsgToTF(currentPose.orientation, qHand);

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
             currentPose.position.x,
             currentPose.position.y,
             currentPose.position.z);
    // Rotate to a vector orthoganal to the velocity.

    // Create the vector representing the direction from the end effector to pole
    // COM
    tf::Vector3 pointAt(solution.targetPose.pose.position.x - currentPose.position.x,
                        solution.targetPose.pose.position.y - currentPose.position.y,
                        solution.targetPose.pose.position.z - currentPose.position.z);

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

    CatchHumanActionServer cha(ros::this_node::getName());
    ros::spin();
}
#endif
