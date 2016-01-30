#include <human_catching/force_controller.hpp>
#include <pluginlib/class_list_macros.h>

using namespace human_catching;

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(force_controller, ForceControllerPlugin,
			human_catching::ForceController,
			pr2_controller_interface::Controller)


bool ForceController::init(pr2_mechanism_model::RobotState *robot,
        ros::NodeHandle &n) {
}

void ForceController::starting() {
}

void ForceController::stopping() {
}
