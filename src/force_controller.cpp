#include <humanoid_catching/force_controller.hpp>
#include <pluginlib/class_list_macros.h>
#include <string>

using namespace humanoid_catching;
using namespace std;

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(force_controller, ForceControllerPlugin,
			humanoid_catching::ForceController,
			pr2_controller_interface::Controller)


void ForceController::commandCB(const geometry_msgs::PoseStampedConstPtr &command) {
    ROS_INFO("Received a new pose command");
    // TODO: Add a mutex around pose_des.
    pose_des = command;
}

bool ForceController::init(pr2_mechanism_model::RobotState *robot,
        ros::NodeHandle &n) {

    string root_name;
    string tip_name;

    if (!n.getParam("root_name", root_name)) {
        ROS_ERROR("No root name given in namespace: %s)", n.getNamespace().c_str());
        return false;
    }

    if (!n.getParam("tip_name", tip_name)) {
        ROS_ERROR("No tip name given in namespace: %s)", n.getNamespace().c_str());
        return false;
    }

    // Construct a chain from the root to the tip and prepare the kinematics
    // Note the joints must be calibrated
    if (!chain.init(robot, root_name, tip_name)) {
        ROS_ERROR("ForceController could not use the chain from '%s' to '%s'", root_name.c_str(), tip_name.c_str());
        return false;
    }

    if (!chain.allCalibrated()) {
        ROS_ERROR("Kinematics chain is not calibrated");
        return false;
    }

    if (!read_only_chain.init(robot, root_name, tip_name)) {
        ROS_ERROR("ForceController could not use the chain from '%s' to '%s'", root_name.c_str(), tip_name.c_str());
        return false;
    }

    // Store the robot handle for later use (to get time)
    robot_state = robot;

    // Construct the kdl solvers in non-realtime
    chain.toKDL(kdl_chain);
    jnt_to_pose_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));

    // KDL::Vector gravity(-9.81, 0.0, 0.0);
    KDL::Vector gravity(0.0, 0.0, 0.0);
    torque_solver.reset(new KDL::ChainIdSolver_RNE(kdl_chain, gravity));

    // Resize (pre-allocate) the variables in non-realtime
    q.resize(kdl_chain.getNrOfJoints());
    qdot.resize(kdl_chain.getNrOfJoints());
    tau.resize(kdl_chain.getNrOfJoints());
    tau_act.resize(kdl_chain.getNrOfJoints());
    J.resize(kdl_chain.getNrOfJoints());
    qdotdot.resize(kdl_chain.getNrOfJoints());
    wrenches.resize(kdl_chain.getNrOfSegments());

    Kd.vel(0) = 0.1;        // Translation x
    Kd.vel(1) = 0.1;        // Translation y
    Kd.vel(2) = 0.1;        // Translation z
    Kd.rot(0) = 0.1;        // Rotation x
    Kd.rot(1) = 0.1;        // Rotation y
    Kd.rot(2) = 0.1;        // Rotation z

    Kp.vel(0) = 0.1;
    Kp.vel(1) = 0.1;
    Kp.vel(2) = 0.1;
    Kp.rot(0) = 0.1;
    Kp.rot(1) = 0.1;
    Kp.rot(2) = 0.1;

    subscriber = n.subscribe("command", 1, &ForceController::commandCB, this);

    return true;
}

void ForceController::starting() {
    ROS_INFO("Starting the ForceController");
}

void ForceController::update()
{
    // Check if there is a current goal
    if (pose_des.get() == NULL) {
        return;
    }

    // Get the current joint positions and velocities
    chain.getPositions(q);
    chain.getVelocities(qdot);

    // Compute the forward kinematics and Jacobian (at this location)
    jnt_to_pose_solver->JntToCart(q, x);
    for (unsigned int i = 0; i < 6; i++){
        xdot(i) = 0;
        for (unsigned int j = 0 ; j < kdl_chain.getNrOfJoints(); j++){
            xdot(i) += J(i, j) * qdot.qdot(j);
        }
    }

    xd.p(0) = pose_des->pose.position.x;
    xd.p(1) = pose_des->pose.position.y;
    xd.p(2) = pose_des->pose.position.z;
    xd.M = KDL::Rotation::Quaternion(pose_des->pose.orientation.x,
                    pose_des->pose.orientation.y,
				      pose_des->pose.orientation.z,
				      pose_des->pose.orientation.w);

    // Calculate a Cartesian restoring force.
    xerr.vel = x.p - xd.p;
    xerr.rot = 0.5 * (xd.M.UnitX() * x.M.UnitX() + xd.M.UnitY() * x.M.UnitY() + xd.M.UnitZ() * x.M.UnitZ());

    // F is a vector of forces/wrenches corresponding to x, y, z, tx,ty,tz,tw
    for(unsigned int i = 0; i < 6; ++i) {
        F(i) = -Kp(i) * xerr(i) - Kd(i) * xdot(i);
    }

    // Convert the force into a set ofjoint torques. Apply the force to the last link.
    wrenches[kdl_chain.getNrOfSegments() - 1] = F;
    int error = torque_solver->CartToJnt(q, qdot.qdot, qdotdot, wrenches, tau);
    if (error != KDL::SolverI::E_NOERROR) {
        ROS_ERROR("Failed to compute inverse dynamics %i", error);
        return;
    }

    // And finally send these torques out
    chain.setEfforts(tau);
}

void ForceController::stopping() {
    ROS_INFO("Stopping the ForceController");
}
