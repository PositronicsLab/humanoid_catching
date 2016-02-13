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
    qdotdot.resize(kdl_chain.getNrOfJoints());

    Kp.vel(0) = 8.0;
    Kp.vel(1) = 8.0;
    Kp.vel(2) = 8.0;
    Kp.rot(0) = 1.0;
    Kp.rot(1) = 1.0;
    Kp.rot(2) = 1.0;

    Kd.vel(0) = 0.05;
    Kd.vel(1) = 0.05;
    Kd.vel(2) = 0.05;
    Kd.rot(0) = 0.01;
    Kd.rot(1) = 0.01;
    Kd.rot(2) = 0.01;

    subscriber = n.subscribe("command", 1, &ForceController::commandCB, this);
    updates = 0;

    controller_state_publisher.reset(new realtime_tools::RealtimePublisher<humanoid_catching::ForceControllerFeedback>(n, "state", 1));
    controller_state_publisher->msg_.requested_joint_efforts.resize(kdl_chain.getNrOfJoints());
    controller_state_publisher->msg_.actual_joint_efforts.resize(kdl_chain.getNrOfJoints());
    controller_state_publisher->msg_.x_error.resize(6);
    controller_state_publisher->msg_.force_desired.resize(6);
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

        // Calculate velocity in operational space from joint space.
        jnt_to_jac_solver->JntToJac(q, J);
        for (unsigned int i = 0; i < 6; i++){
            xdot(i) = 0;
            for (unsigned int j = 0 ; j < kdl_chain.getNrOfJoints(); j++){
                xdot(i) += J(i, j) * qdot.qdot(j);
            }
        }

        // Convert the desired pose from ROS to KDL
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
        // Reduce velocity as the position approaches the desired position.
        for(unsigned int i = 0; i < 6; ++i) {
            F(i) = -Kp(i) * xerr(i) - Kd(i) * xdot(i);
        }

        // Convert the force into a set of joint torques
        // tau_ is a vector of joint torques q1...qn
        for (unsigned int i = 0 ; i < kdl_chain.getNrOfJoints(); i++) {
            // Iterate through the vector. Every joint torque is contributed to
            // by the Jacobian Transpose (note the index switching in J access) times
            // the desired force (from impedance OR explicit force)
            tau(i) = 0;
            for (unsigned int j = 0 ; j < 6 ; j++) {
                tau(i) += J(j, i) * F(j);
            }
        }

        // Send the torques to the robot
        chain.setEfforts(tau);
    }

    // Send controller feedback
    if (updates % 10 == 0 && controller_state_publisher && controller_state_publisher->trylock()) {
        chain.getEfforts(tau_act);
        double eff_err = 0;
        for (unsigned int i = 0; i < kdl_chain.getNrOfJoints(); i++) {
            eff_err += (tau(i) - tau_act(i)) * (tau(i) - tau_act(i));
            controller_state_publisher->msg_.requested_joint_efforts[i] = tau(i);
            controller_state_publisher->msg_.actual_joint_efforts[i] = tau_act(i);
        }

        double pose_sq_err = 0;
        for (unsigned int i = 0; i < 6; ++i) {
            pose_sq_err += xerr[i] * xerr[i];
        }

        double force_desired_sq = 0;
        for (unsigned int i = 0; i < 6; ++i) {
            force_desired_sq += F[i] * F[i];
        }

        controller_state_publisher->msg_.header.stamp = robot_state->getTime();
        controller_state_publisher->msg_.effort_sq_error = eff_err;
        controller_state_publisher->msg_.pose_sq_error = pose_sq_err;
        controller_state_publisher->msg_.force_desired_sq = force_desired_sq;

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
        for (int i = 0; i < 6; ++i) {
            controller_state_publisher->msg_.x_error[i] = xerr(i);
        }
        for (int i = 0; i < 6; ++i) {
            controller_state_publisher->msg_.force_desired[i] = F(i);
        }
        controller_state_publisher->unlockAndPublish();
    }
}

void ForceController::stopping() {
    ROS_INFO("Stopping the ForceController");
}
