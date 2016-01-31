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

    if (!read_only_chain.init(robot, root_name, tip_name)) {
        ROS_ERROR("ForceController could not use the chain from '%s' to '%s'", root_name.c_str(), tip_name.c_str());
        return false;
    }

    // Store the robot handle for later use (to get time)
    robot_state = robot;

    // Construct the kdl solvers in non-realtime
    chain.toKDL(kdl_chain);
    jnt_to_pose_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain));

    // Resize (pre-allocate) the variables in non-realtime
    q.resize(kdl_chain.getNrOfJoints());
    qdot.resize(kdl_chain.getNrOfJoints());
    tau.resize(kdl_chain.getNrOfJoints());
    tau_act.resize(kdl_chain.getNrOfJoints());
    J.resize(kdl_chain.getNrOfJoints());

    Kd.vel(0) = 0.0;        // Translation x
    Kd.vel(1) = 0.0;        // Translation y
    Kd.vel(2) = 0.0;        // Translation z
    Kd.rot(0) = 0.0;        // Rotation x
    Kd.rot(1) = 0.0;        // Rotation y
    Kd.rot(2) = 0.0;        // Rotation z

    subscriber = n.subscribe("command", 1, &ForceController::commandCB, this);

    return true;
}

void ForceController::starting() {
    ROS_INFO("Starting the ForceController");
}

void ForceController::update()
{
    if (pose_des.get() == NULL) {
        ROS_INFO("No goal position set");
        return;
    }

    // Get the current joint positions and velocities
    chain.getPositions(q);
    chain.getVelocities(qdot);

    // Compute the forward kinematics and Jacobian (at this location)
    jnt_to_pose_solver->JntToCart(q, x);
    jnt_to_jac_solver->JntToJac(q, J);

    for (unsigned int i = 0; i < 6; i++){
        xdot(i) = 0;
        for (unsigned int j = 0 ; j < kdl_chain.getNrOfJoints(); j++){
            xdot(i) += J(i, j) * qdot.qdot(j);
        }
    }

    Kp.vel(0) = 0;
    Kp.vel(1) = 0;
    Kp.vel(2) = 0;
    Kp.rot(0) = 0;
    Kp.rot(1) = 0;
    Kp.rot(2) = 0;

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
    F(0) = -Kp(0) * xerr(0) - Kd(0) * xdot(0);
    F(1) = -Kp(1) * xerr(1) - Kd(1) * xdot(1);
    F(2) = -Kp(2) * xerr(2) - Kd(2) * xdot(2);
    F(3) = -Kp(3) * xerr(3) - Kd(3) * xdot(3);
    F(4) = -Kp(4) * xerr(4) - Kd(4) * xdot(4);
    F(5) = -Kp(5) * xerr(5) - Kd(5) * xdot(5);

    // Convert the force into a set of joint torques
    // tau is a vector of joint torques q1...qn
    for (unsigned int i = 0 ; i < kdl_chain.getNrOfJoints() ; i++) {
        // Iterate through the vector. Every joint torque is contributed to
        // by the Jacobian Transpose (note the index switching in J access) times
        // the desired force (from impedance OR explicit force)

        // If a desired end effector wrench is specified, there is no position control on that dof
        // if a wrench is not specified, then there is impedance based (basically p-gain)
        // position control on that dof
        tau(i) = 0;
        for (unsigned int j = 0 ; j < 6 ; j++) {
            tau(i) += J(j, i) * F(j);
        }
    }

    // And finally send these torques out
    chain.setEfforts(tau);
}

void ForceController::stopping() {
    ROS_INFO("Stopping the ForceController");
}
