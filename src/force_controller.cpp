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
    if (command->header.frame_id != root_name) {
        ROS_ERROR("Pose commands must be in the %s frame. Command was in the %s frame.",
                  root_name.c_str(), command->header.frame_id.c_str());
        return;
    }
    pose_des.set(command);
}

bool ForceController::init(pr2_mechanism_model::RobotState *robot,
        ros::NodeHandle &n) {

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
    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain));

    // Gravity zeroed out as PR2 mechanically compensates for gravity in the arms
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

    Kd.vel(0) = 0.2;        // Translation x
    Kd.vel(1) = 0.2;        // Translation y
    Kd.vel(2) = 0.2;        // Translation z
    Kd.rot(0) = 0.2;        // Rotation x
    Kd.rot(1) = 0.2;        // Rotation y
    Kd.rot(2) = 0.2;        // Rotation z

    Kp.vel(0) = 0.6;
    Kp.vel(1) = 0.6;
    Kp.vel(2) = 0.6;
    Kp.rot(0) = 0.6;
    Kp.rot(1) = 0.6;
    Kp.rot(2) = 0.6;

    subscriber = n.subscribe("command", 1, &ForceController::commandCB, this);
    updates = 0;

    controller_state_publisher.reset(new realtime_tools::RealtimePublisher<humanoid_catching::ForceControllerFeedback>(n, "state", 1));
    controller_state_publisher->msg_.requested_joint_efforts.resize(kdl_chain.getNrOfJoints());
    controller_state_publisher->msg_.actual_joint_efforts.resize(kdl_chain.getNrOfJoints());
    controller_state_publisher->msg_.pose.header.frame_id = root_name;
    return true;
}

void ForceController::starting() {
    ROS_INFO("Starting the ForceController");
}

void ForceController::update()
{
    // Check if there is a current goal
    boost::shared_ptr<const geometry_msgs::PoseStamped> pose_des_ptr;
    pose_des.get(pose_des_ptr);
    updates++;

    // Get the current joint positions and velocities
    chain.getPositions(q);
    chain.getVelocities(qdot);

    // Compute the forward kinematics and Jacobian (at this location)
    jnt_to_pose_solver->JntToCart(q, x);

    if (pose_des_ptr.get() != NULL) {

        jnt_to_jac_solver->JntToJac(q, J);
        for (unsigned int i = 0; i < 6; i++){
            xdot(i) = 0;
            for (unsigned int j = 0 ; j < kdl_chain.getNrOfJoints(); j++){
                xdot(i) += J(i, j) * qdot.qdot(j);
            }
        }

        xd.p(0) = pose_des_ptr->pose.position.x;
        xd.p(1) = pose_des_ptr->pose.position.y;
        xd.p(2) = pose_des_ptr->pose.position.z;
        xd.M = KDL::Rotation::Quaternion(pose_des_ptr->pose.orientation.x,
                    pose_des_ptr->pose.orientation.y,
                    pose_des_ptr->pose.orientation.z,
                    pose_des_ptr->pose.orientation.w);

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

    if (updates % 10 == 0 && controller_state_publisher && controller_state_publisher->trylock()) {
        chain.getEfforts(tau_act);
        double eff_err = 0;
        for (unsigned int i = 0; i < kdl_chain.getNrOfJoints(); i++) {
            eff_err += (tau(i) - tau_act(i))*(tau(i) - tau_act(i));
            controller_state_publisher->msg_.requested_joint_efforts[i] = tau(i);
            controller_state_publisher->msg_.actual_joint_efforts[i] = tau_act(i);
        }

        double pose_sq_err = 0;
        for (unsigned int i = 0; i < 6; ++i) {
            pose_sq_err += xerr[i] * xerr[i];
        }

        controller_state_publisher->msg_.header.stamp = robot_state->getTime();
        controller_state_publisher->msg_.effort_sq_error = eff_err;
        controller_state_publisher->msg_.pose_sq_error = pose_sq_err;

        if(pose_des_ptr != NULL) {
            controller_state_publisher->msg_.goal.header = pose_des_ptr->header;
            controller_state_publisher->msg_.goal.pose = pose_des_ptr->pose;
        }

        controller_state_publisher->msg_.header.stamp = robot_state->getTime();
        controller_state_publisher->msg_.pose.pose.position.x = x.p(0);
        controller_state_publisher->msg_.pose.pose.position.y = x.p(1);
        controller_state_publisher->msg_.pose.pose.position.z = x.p(2);

        double qx, qy, qz, qw;
        x.M.GetQuaternion(qx, qy, qz, qw);
        controller_state_publisher->msg_.pose.pose.orientation.x = qx;
        controller_state_publisher->msg_.pose.pose.orientation.y = qy;
        controller_state_publisher->msg_.pose.pose.orientation.z = qz;
        controller_state_publisher->msg_.pose.pose.orientation.w = qw;
        controller_state_publisher->unlockAndPublish();
    }
}

void ForceController::stopping() {
    ROS_INFO("Stopping the ForceController");
}
