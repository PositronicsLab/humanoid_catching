#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <humanoid_catching/CatchHumanAction.h>
#include <kinematics_cache/IKQuery.h>
#include <humanoid_catching/PredictFall.h>
#include <actionlib/client/simple_action_client.h>
#include <humanoid_catching/Move.h>
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

namespace
{
using namespace std;
using namespace humanoid_catching;

typedef actionlib::SimpleActionServer<humanoid_catching::CatchHumanAction> Server;
typedef vector<kinematics_cache::IK> IKList;

static const string ARMS[] = {"left_arm", "right_arm"};
static const string ARM_TOPICS[] = {"l_arm_force_controller/command", "r_arm_force_controller/command"};
static const string ARM_GOAL_VIZ_TOPICS[] = {"/catch_human_action_server/movement_goal/left_arm", "/catch_human_action_server/movement_goal/right_arm"};
static const double MAX_VELOCITY = 100;
static const double MAX_ACCELERATION = 100;
static const double MAX_EFFORT = 100;

//! Tolerance of time to be considered in contact
static const ros::Duration CONTACT_TIME_TOLERANCE = ros::Duration(0.1);

static const ros::Duration SEARCH_RESOLUTION(0.10);
static const double pi = boost::math::constants::pi<double>();

struct Limit
{
    Limit() : velocity(0.0), acceleration(0.0), effort(0.0) {}
    Limit(double aVelocity, double aAcceleration, double aEffort) : velocity(aVelocity), acceleration(aAcceleration), effort(aEffort) {}
    double velocity;
    double acceleration;
    double effort;
};
typedef std::map<std::string, Limit> LimitMapType;

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
    bool armsSolved[2];
    bool feasable;
    geometry_msgs::PoseStamped pose;
    ros::Duration goalTime;
    ros::Duration delta;

    Solution()
    {
        armsSolved[0] = armsSolved[1] = false;
        feasable = false;
    }
};

static geometry_msgs::Pose applyTransform(const geometry_msgs::PoseStamped& pose, const tf::StampedTransform transform)
{
    tf::Stamped<tf::Pose> tfPose;
    tf::poseStampedMsgToTF(pose, tfPose);
    tf::Pose tfGoal = transform * tfPose;
    geometry_msgs::Pose msgGoal;
    tf::poseTFToMsg(tfGoal, msgGoal);
    return msgGoal;
}

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

    //! Cached IK client.
    ros::ServiceClient ik;

    //! Cached balancing client
    ros::ServiceClient balancer;

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

    //! Kinematic model of the robot
    robot_model::RobotModelPtr kinematicModel;

    //! Ravelin dynamic body
    boost::shared_ptr<Ravelin::RCArticulatedBodyd> body;

    //! Record the indexes into the body for the two sets of arm joints
    map<string, pair<int, int> > jointIndices;

    //! Create messages that are used to published feedback/result
    humanoid_catching::CatchHumanFeedback feedback;
    humanoid_catching::CatchHumanResult result;
private:
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        ROS_DEBUG("Received a joint states message");
        for(unsigned int i = 0; i < msg->name.size(); ++i)
        {
            StateMapType::iterator iter = jointStates.find(msg->name[i]);
            if (iter != jointStates.end())
            {
                iter->second.position = msg->position[i];
                iter->second.velocity = msg->velocity[i];
            }
        }
    }

public:
    CatchHumanActionServer(const string& name) :
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

        ROS_INFO("Waiting for kinematics_cache/ik service");
        ros::service::waitForService("/kinematics_cache/ik");
        ik = nh.serviceClient<kinematics_cache::IKQuery>("/kinematics_cache/ik", true /* persistent */);

        ROS_INFO("Waiting for /balancer/torques service");
        ros::service::waitForService("/balancer/torques");
        balancer = nh.serviceClient<humanoid_catching::CalculateTorques>("/balancer/torques", true /* persistent */);

        // Initialize arm clients
        ROS_INFO("Initializing arm command publishers");
        for (unsigned int i = 0; i < boost::size(ARMS); ++i)
        {
            armCommandPubs.push_back(nh.advertise<humanoid_catching::Move>(ARM_TOPICS[i], 1, false));
            goalPubs.push_back(nh.advertise<geometry_msgs::PointStamped>(ARM_GOAL_VIZ_TOPICS[i], 10, true));
        }

        trialGoalPub = nh.advertise<geometry_msgs::PointStamped>("/catch_human_action_server/movement_goal_trials", 1);

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

        ROS_INFO("Creating body");
        body = boost::shared_ptr<Ravelin::RCArticulatedBodyd>(new Ravelin::RCArticulatedBodyd());
        body->set_links_and_joints(links, joints);
        body->set_floating_base(false);
        ROS_INFO("Body created correctly");

        for (unsigned int k = 0; k < boost::size(ARMS); ++k)
        {
            const robot_model::JointModelGroup* jointModelGroup =  kinematicModel->getJointModelGroup(ARMS[k]);

            // Determine the indexes for the arms within the body joints in Ravelin
            int lowerIndex = -1;
            int upperIndex = -1;
            int curr = 0;
            for (int l = 0; l < joints.size(); ++l)
            {
                if (joints[l]->joint_id == jointModelGroup->getActiveJointModelNames()[0])
                {
                    assert(lowerIndex == -1);
                    lowerIndex = curr;
                }
                if (joints[l]->joint_id == jointModelGroup->getActiveJointModelNames().back())
                {
                    assert(upperIndex == -1);
                    upperIndex = curr;
                }
                curr += joints[l]->num_dof();
            }

            assert(lowerIndex != -1);
            assert(upperIndex != -1);

            jointIndices[ARMS[k]] = pair<int, int>(lowerIndex, upperIndex);

            ROS_INFO("Loading joint limits for arm %s", ARMS[k].c_str());
            for (unsigned int i = 0; i < jointModelGroup->getActiveJointModels().size(); ++i)
            {
                const string& jointName = jointModelGroup->getActiveJointModels()[i]->getName();
                jointNames[ARMS[k]].push_back(jointName);

                // Set default position and velocity
                jointStates[jointName] = State(0.0, 0.0);

                const string prefix = "robot_description_planning/joint_limits/" + jointName + "/";
                ROS_DEBUG_NAMED("catch_human_action_server", "Loading velocity and acceleration limits for joint %s", jointName.c_str());

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
                const urdf::Joint* ujoint = urdfModel->getJoint(jointName).get();
                if (ujoint != NULL && ujoint->limits)
                {
                    max_effort = ujoint->limits->effort;
                    ROS_DEBUG_NAMED("catch_human_action_server", "Setting max effort to %f for %s", max_effort, jointName.c_str());
                }
                else
                {
                    ROS_DEBUG_NAMED("catch_human_action_server", "Setting max effort to default for %s", jointName.c_str());
                    max_effort = MAX_EFFORT;
                }
                jointLimits[jointName] = Limit(max_velocity, max_acc, max_effort);
            }
        }

        ROS_INFO("Completed initializing the joint limits");

        jointStatesSub.reset(new message_filters::Subscriber<sensor_msgs::JointState>(nh, "/joint_states", 5));
        jointStatesSub->registerCallback(boost::bind(&CatchHumanActionServer::jointStatesCallback, this, _1));

        ROS_INFO("Starting the action server");
        as.registerPreemptCallback(boost::bind(&CatchHumanActionServer::preempt, this));
        as.start();
        ROS_INFO("Catch human action server initialized successfully");
    }

private:

    void preempt()
    {
        // Currently no way to cancel force controllers.
        as.setPreempted();
    }

    void visualizeGoal(const geometry_msgs::PoseStamped& goal, unsigned int armIndex) const
    {
        if (goalPubs[armIndex].getNumSubscribers() > 0)
        {
            geometry_msgs::PointStamped point;
            point.header = goal.header;
            point.point = goal.pose.position;
            goalPubs[armIndex].publish(point);
        }
    }

    static geometry_msgs::PointStamped poseToPoint(const geometry_msgs::PoseStamped pose)
    {
        geometry_msgs::PointStamped point;
        point.point = pose.pose.position;
        point.header = pose.header;
        return point;
    }

    // TODO: This shares a lot of code with the moveit_plugin
    // TODO: Incorporate current velocity
    ros::Duration calcExecutionTime(const string& group, const vector<double>& solution)
    {

        double longestTime = 0.0;
        for(unsigned int i = 0; i < solution.size(); ++i)
        {
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
            if (d <= max_tri_distance)
            {
                t = sqrt(d / limits.acceleration);
                ROS_DEBUG_NAMED("catch_human_action_server", "Triangular solution for t is %f", t);
            }
            else
            {
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

    static unsigned int numArmsSolved(const Solution& solution)
    {
        unsigned int numArmsSolved = 0;
        for (unsigned int k = 0; k < boost::size(ARMS); ++k)
        {
            if (solution.armsSolved[k])
            {
                numArmsSolved++;
            }
        }
        return numArmsSolved;
    }

    geometry_msgs::Pose tfFrameToPose(const string& tfFrame, const std_msgs::Header& header) const
    {
        tf::StampedTransform tfStampedTransform;
        tf.lookupTransform("/map", tfFrame, header.stamp, tfStampedTransform);
        geometry_msgs::TransformStamped stampedTransform;
        transformStampedTFToMsg(tfStampedTransform, stampedTransform);
        geometry_msgs::Pose pose;
        pose.position.x = stampedTransform.transform.translation.x;
        pose.position.y = stampedTransform.transform.translation.y;
        pose.position.z = stampedTransform.transform.translation.z;
        pose.orientation = stampedTransform.transform.rotation;
        return pose;
    }

    /**
     * Determine whether the robot is currently in contact with the humanoid.
     */
    bool isRobotInContact(const humanoid_catching::PredictFall::Response& fall) const
    {
        for (vector<FallPoint>::const_iterator i = fall.points.begin(); i != fall.points.end(); ++i)
        {
            if (i->time > CONTACT_TIME_TOLERANCE)
            {
                break;
            }
            for (vector<Contact>::const_iterator c = i->contacts.begin(); c != i->contacts.end(); ++c)
            {
                if (c->is_in_contact)
                {
                    return true;
                }
            }
        }
        return false;
    }

    const vector<FallPoint>::const_iterator findContact(const humanoid_catching::PredictFall::Response& fall, unsigned int arm) const
    {
        for (vector<FallPoint>::const_iterator i = fall.points.begin(); i != fall.points.end(); ++i)
        {
            if (i->time > CONTACT_TIME_TOLERANCE)
            {
                break;
            }
            if (i->contacts[arm].is_in_contact)
            {
                return i;
            }
        }
        return fall.points.end();
    }

    bool endEffectorPositions(const std_msgs::Header& header, vector<geometry_msgs::Pose>& poses) const
    {
        ROS_DEBUG("Waiting for wrist transforms");
        if(!tf.waitForTransform("/map", "/l_wrist_roll_link", header.stamp, ros::Duration(1)) ||
                !tf.waitForTransform("/map", "r_wrist_roll_link", header.stamp, ros::Duration(1)))
        {
            ROS_WARN("Failed to get wrist transform");
            return false;
        }

        poses.resize(2);
        poses[0] = tfFrameToPose("/l_wrist_roll_link", header);
        poses[1] = tfFrameToPose("/r_wrist_roll_link", header);
        return true;
    }

    void sendTorques(const unsigned int arm, const vector<double>& torques) {
        // Execute the movement
        ROS_INFO("Dispatching torque command for arm %s", ARMS[arm].c_str());
        humanoid_catching::Move command;
        command.header.stamp = ros::Time::now();
        command.header.frame_id = "/torso_lift_link";
        command.has_torques = true;
        command.torques = torques;
        armCommandPubs[arm].publish(command);
    }

    void execute(const humanoid_catching::CatchHumanGoalConstPtr& goal)
    {

        ROS_INFO("Catch procedure initiated");

        if(!as.isActive() || as.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Catch human action canceled before started");
            return;
        }

        ros::WallTime startWallTime = ros::WallTime::now();
        ros::Time startRosTime = ros::Time::now();

        humanoid_catching::PredictFall predictFall;
        predictFall.request.header = goal->header;
        predictFall.request.pose = goal->pose;
        predictFall.request.velocity = goal->velocity;
        predictFall.request.accel = goal->accel;

        // Wait for the wrist transforms
        if (!endEffectorPositions(goal->header, predictFall.request.end_effectors))
        {
            as.setAborted();
            return;
        }

        ROS_DEBUG("Predicting fall");
        if (!fallPredictor.call(predictFall))
        {
            ROS_WARN("Fall prediction failed");
            as.setAborted();
            return;
        }
        ROS_DEBUG("Fall predicted successfully");

        // Transform to the base_frame of the human (and therefore the obstacle) for IK, which is torso_lift_link
        ROS_DEBUG("Waiting for /torso_lift_link transform");
        if(!tf.waitForTransform(predictFall.response.header.frame_id, "/torso_lift_link", predictFall.response.header.stamp, ros::Duration(15)))
        {
            ROS_WARN("Failed to get transform from %s to /torso_lift_link", predictFall.response.header.frame_id.c_str());
            as.setAborted();
            return;
        }

        tf::StampedTransform goalToTorsoTransform;
        tf.lookupTransform("/torso_lift_link", predictFall.response.header.frame_id, predictFall.response.header.stamp, goalToTorsoTransform);

        ROS_DEBUG("Transforms acquired");

        // Determine if the robot is currently in contact
        if (isRobotInContact(predictFall.response))
        {
            // Determine which arms to execute balancing for.
            for (unsigned int i = 0; i < boost::size(ARMS); ++i)
            {
                const vector<FallPoint>::const_iterator fallPoint = findContact(predictFall.response, i);
                if (fallPoint == predictFall.response.points.end())
                {
                    ROS_INFO("Arm %s is not in contact", ARMS[i].c_str());
                    vector<double> zeroTorques;
                    zeroTorques.resize(7);
                    sendTorques(i, zeroTorques);
                    continue;
                }
                else
                {
                    ROS_INFO("Robot is in contact. Executing balancing for arm %s", ARMS[i].c_str());
                }

                humanoid_catching::CalculateTorques calcTorques;
                calcTorques.request.body_velocity = fallPoint->velocity;
                calcTorques.request.body_inertia_matrix = predictFall.response.inertia_matrix;
                calcTorques.request.body_mass = predictFall.response.body_mass;
                calcTorques.request.time_delta = ros::Duration(0.001);
                calcTorques.request.body_com = fallPoint->pose;
                calcTorques.request.contact_position = fallPoint->contacts[i].position;
                calcTorques.request.contact_normal = fallPoint->contacts[i].normal;
                calcTorques.request.ground_contact = fallPoint->ground_contact;

                // Get the inertia matrix
                Ravelin::MatrixNd robotInertiaMatrix;
                body->get_generalized_inertia(robotInertiaMatrix);

                Ravelin::MatrixNd armMatrix;
                robotInertiaMatrix.get_sub_mat(jointIndices[ARMS[i]].first, jointIndices[ARMS[i]].second + 1,
                                               jointIndices[ARMS[i]].first, jointIndices[ARMS[i]].second + 1, armMatrix);

                calcTorques.request.robot_inertia_matrix = vector<double>(armMatrix.data(), armMatrix.data() + armMatrix.size());

                // Get the jacobian
                robot_state::RobotState currentRobotState = planningScene->getPlanningScene()->getCurrentState();
                const robot_model::JointModelGroup* jointModelGroup = kinematicModel->getJointModelGroup(ARMS[i]);
                Eigen::MatrixXd jacobian = currentRobotState.getJacobian(jointModelGroup);

                // Set current velocities
                const vector<string>& jointModelNames = jointModelGroup->getActiveJointModelNames();
                calcTorques.request.joint_velocity.resize(jointModelNames.size());
                for (unsigned int j = 0; j < jointModelNames.size(); ++j) {
                    calcTorques.request.joint_velocity[j] = jointStates[jointModelNames[j]].velocity;
                }

                // Convert to raw type
                calcTorques.request.jacobian_matrix.resize(jacobian.rows() * jacobian.cols());
                Eigen::Map<Eigen::MatrixXd>(&calcTorques.request.jacobian_matrix[0], jacobian.rows(), jacobian.cols()) = jacobian;

                // Set the joint limits
                calcTorques.request.torque_limits.resize(jointModelNames.size());
                calcTorques.request.velocity_limits.resize(jointModelNames.size());
                for (unsigned int j = 0; j < jointModelNames.size(); ++j)
                {
                    double maxEffort = jointLimits[jointModelNames[j]].effort;
                    calcTorques.request.torque_limits[j].minimum = -maxEffort;
                    calcTorques.request.torque_limits[j].maximum = maxEffort;

                    double maxVelocity = jointLimits[jointModelNames[j]].velocity;
                    calcTorques.request.velocity_limits[j].minimum = -maxVelocity;
                    calcTorques.request.velocity_limits[j].maximum = maxVelocity;
                }

                ROS_DEBUG("Calculating torques");
                if (!balancer.call(calcTorques))
                {
                    ROS_WARN("Calculating torques failed");
                    continue;
                }
                ROS_DEBUG("Torques calculated successfully");
                sendTorques(i, calcTorques.response.torques);
            }
        }
        else
        {
            // We now have a projected time/position path. Search the path for acceptable times.
            vector<Solution> solutions;

            // Epsilon causes us to always select a position in front of the fall.
            ros::Duration lastTime;

            for (vector<FallPoint>::const_iterator i = predictFall.response.points.begin(); i != predictFall.response.points.end(); ++i)
            {

                // Determine if we should search this point.
                if (lastTime != ros::Duration(0) && i->time - lastTime < SEARCH_RESOLUTION)
                {
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

                if (trialGoalPub.getNumSubscribers() > 0)
                {
                    trialGoalPub.publish(poseToPoint(transformedPose));
                }

                // Lookup the IK solution
                kinematics_cache::IKQuery ikQuery;
                ikQuery.request.pose = transformedPose;

                if (!ik.call(ikQuery))
                {
                    ROS_DEBUG("Failed to find IK solution for arms");
                    continue;
                }

                // Check for self-collision with this solution
                planning_scene::PlanningScenePtr currentScene = planningScene->getPlanningScene();

                ROS_DEBUG("Received %lu results from IK query", ikQuery.response.results.size());
                for (IKList::iterator j = ikQuery.response.results.begin(); j != ikQuery.response.results.end(); ++j)
                {

                    // We estimate that the robot will move to the position in the cache. This may be innaccurate and there
                    // may be another position that is not in collision.
                    // Note: The allowed collision matrix should prevent collisions between the arms
                    robot_state::RobotState currentRobotState = currentScene->getCurrentState();
                    currentRobotState.setVariablePositions(jointNames[j->group], j->positions);
                    if (currentScene->isStateColliding(currentRobotState, j->group, false))
                    {
                        ROS_DEBUG("State in collision.");
                        continue;
                    }

                    possibleSolution.feasable = true;

                    // We want to search for the fastest arm movement. The idea is that the first arm there should slow the human
                    // down and give the second arm time to arrive.
                    possibleSolution.delta = max(possibleSolution.delta, ros::Duration(i->time) - calcExecutionTime(j->group, j->positions));

                    for (unsigned int k = 0; k < boost::size(ARMS); ++k)
                    {
                        if (j->group == ARMS[k])
                        {
                            possibleSolution.armsSolved[k] = true;
                        }
                    }
                }

                if (possibleSolution.feasable)
                {
                    solutions.push_back(possibleSolution);
                }
                else
                {
                    ROS_DEBUG("Solution was not feasible");
                }
            }

            if (solutions.empty())
            {
                ROS_WARN("No possible catch positions");
                as.setAborted();
                return;
            }

            ROS_INFO("Selecting optimal position from %lu poses", solutions.size());

            // Select the point which is reachable most quicly
            ros::Duration highestDeltaTime = ros::Duration(-1000);
            unsigned int highestArmsSolved = 0;

            vector<Solution>::iterator bestSolution = solutions.end();
            for (vector<Solution>::iterator solution = solutions.begin(); solution != solutions.end(); ++solution)
            {

                // Always prefer higher armed solutions
                if (numArmsSolved(*solution) > highestArmsSolved || numArmsSolved(*solution) == highestArmsSolved && solution->delta > highestDeltaTime)
                {
                    highestArmsSolved = numArmsSolved(*solution);
                    highestDeltaTime = solution->delta;
                    ROS_DEBUG("Selected a pose with more arms solved or a higher delta time. Pose is %f %f %f, arms solved is %u, and delta is %f", solution->pose.pose.position.x,
                              solution->pose.pose.position.y, solution->pose.pose.position.z, highestArmsSolved, highestDeltaTime.toSec());
                    bestSolution = solution;
                }
            }

            if (bestSolution == solutions.end())
            {
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
            for (unsigned int i = 0; i < armCommandPubs.size(); ++i)
            {
                if (bestSolution->armsSolved[i])
                {
                    ROS_INFO("Publishing command for arm %s.", ARMS[i].c_str());
                    humanoid_catching::Move command;
                    visualizeGoal(bestSolution->pose, i);
                    command.header = bestSolution->pose.header;
                    command.target = bestSolution->pose.pose;
                    command.obstacle = obstacle.pose;
                    command.has_obstacle = false;
                    command.point_at_target = true;
                    armCommandPubs[i].publish(command);
                }
                else
                {
                    ROS_INFO("Skipping arm %s due to no solution.", ARMS[i].c_str());
                }
            }
        }

        ROS_INFO("Reaction time was %f(s) wall time and %f(s) clock time", ros::WallTime::now().toSec() - startWallTime.toSec(),
        ros::Time::now().toSec() - startRosTime.toSec());
        as.setSucceeded();
    }
};
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "catch_human_action");

    CatchHumanActionServer cha(ros::this_node::getName());
    ros::spin();
}
