#ifndef FORCE_CONTROLLER_HPP
#define FORCE_CONTROLLER_HPP

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <pr2_mechanism_model/robot.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_box.h>

#include <humanoid_catching/ForceControllerFeedback.h>

// Code inspired by and based on ee_imped_controller
namespace humanoid_catching {

  class ForceController : public pr2_controller_interface::Controller {
  private:

    //! Subscriber for command updates
    ros::Subscriber subscriber;

    //! The current robot state
    pr2_mechanism_model::RobotState* robot_state;

    //! The chain of links and joints in PR2 language for reference and commanding
    pr2_mechanism_model::Chain chain;

    //! The chain of links and joints in PR2 language for reference only
    pr2_mechanism_model::Chain read_only_chain;

    //! The chain of links and joints in KDL language
    KDL::Chain kdl_chain;

    //! KDL Solver performing the joint angles to Cartesian pose calculation
    boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver;

    //! KDL Solver performing the force to torque solver
    boost::scoped_ptr<KDL::ChainIdSolver_RNE> torque_solver;

    /// KDL Solver performing the joint angles to Jacobian calculation
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;

    //! Joint positions
    KDL::JntArray  q;

    //! Joint velocities
    KDL::JntArrayVel  qdot;

    //! Joint accelerations
    KDL::JntArray qdotdot;

    //! Joint torques
    KDL::JntArray  tau;

    KDL::JntArray tau_act;

    //! Tip pose
    KDL::Frame     x;

    //! Tip desired pose
    KDL::Frame     xd;

    //! Cartesian error
    KDL::Twist     xerr;

    //! Cartesian velocity
    KDL::Twist     xdot;

    //! Cartesian effort
    KDL::Wrench    F;

    //! Wrenches supplied to inverse dynamics solver
    KDL::Wrenches wrenches;

    //! Jacobian
    KDL::Jacobian  J;

    //! Proportional gains
    KDL::Twist     Kp;

    //! Derivative gains
    KDL::Twist     Kd;

    //! State publisher, published every 10 updates
    boost::scoped_ptr<realtime_tools::RealtimePublisher<humanoid_catching::ForceControllerFeedback> > controller_state_publisher;

    //! Goal
    realtime_tools::RealtimeBox<boost::shared_ptr<const geometry_msgs::PoseStamped> > pose_des;

    //! Current update count
    int updates;

    //! Name of the root of the kinematics chain. This is the frame for all commands.
    std::string root_name;

    //! Name of the tip in the kinematics chain.
    std::string tip_name;

    //! Callback when a new goal is received
    void commandCB(const geometry_msgs::PoseStampedConstPtr &command);

  public:
    /**
     * \brief Controller initialization in non-realtime
     *
     * Initializes the controller on first startup.  Prepares the
     * kinematics, pre-allocates the variables, subscribes to
     * command and advertises state.
     *
     * @param robot The current robot state
     * @param n A node handle for subscribing and advertising topics
     *
     * @return True on successful initialization, false otherwise
     *
     */
    bool init(pr2_mechanism_model::RobotState *robot,
	      ros::NodeHandle &n);
    /**
     * \brief Controller startup in realtime
     *
     * Resets the controller to prepare for a new goal.  Sets the desired
     * position and orientation to be the current position and orientation
     * and sets the joints to have maximum stiffness so that they hold
     * their current position.
     */
    void starting();

    /**
     * \brief Controller update loop in realtime
     *
     * A PD controller for achieving the trajectory.
     * Uses EECartImpedControlClass::sampleInterpolation to
     * find the point that should be achieved on the current timestep.
     * Converts this point (which is in Cartesian coordinates) to joint
     * angles and uses a PD update (in the case of stiffness) to send
     * to the joints the correct force.
     *
     */
    void update();

    /**
     * \brief Controller stopping in realtime
     *
     * Calls EECartImpedControlClass::starting() to lock the joints into
     * their current position.
     *
     */
    void stopping();

  };
}

#endif
