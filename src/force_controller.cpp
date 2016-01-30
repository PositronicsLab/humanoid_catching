#include <human_catching/force_controller.hpp>
#include <pluginlib/class_list_macros.h>
#include <string>

using namespace human_catching;
using namespace std;

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(force_controller, ForceControllerPlugin,
			human_catching::ForceController,
			pr2_controller_interface::Controller)


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

    return true;
}

void ForceController::starting() {
}

void ForceController::stopping() {
}
