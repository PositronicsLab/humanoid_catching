#include <ros/ros.h>
#include <humanoid_catching/CalculateTorques.h>
#include <Moby/qpOASES.h>
#include <Ravelin/VectorNd.h>
#include <Ravelin/MatrixNd.h>
#include <Ravelin/Quatd.h>
#include <Ravelin/Opsd.h>
#include <Moby/qpOASES.h>
#include <Ravelin/LinAlgd.h>

namespace {
using namespace std;
using namespace humanoid_catching;
using namespace Ravelin;

static const double GRAVITY = -9.81;
static const unsigned int POLE_DOF = 6;
static const double EPSILON = 0.001;

class Balancer {
private:
    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Linear algebra object
    LinAlgd linAlgd;

    //! Balancer Service
    ros::ServiceServer balancerService;

public:
   Balancer() :
     pnh("~") {

       balancerService = nh.advertiseService("/balancer/torques",
            &Balancer::calculateTorques, this);
	}

private:

    /**
     * Convert a geometry_msgs::Vector3 to a Ravelin::Vector3d
     * @param v3 geometry_msgs::Vector3
     * @return Ravelin Vector3d
     */
    static Vector3d to_vector(const geometry_msgs::Vector3& v3) {
        Vector3d v;
        v[0] = v3.x;
        v[1] = v3.y;
        v[2] = v3.z;
        return v;
    }

    /**
     * Convert a geometry_msgs::Point to a Ravelin::Vector3d
     * @param p geometry_msgs::Point
     * @return Ravelin Vector3d
     */
    static Vector3d to_vector(const geometry_msgs::Point& p) {
        Vector3d v;
        v[0] = p.x;
        v[1] = p.y;
        v[2] = p.z;
        return v;
    }

    /**
     * Convert a vector<double> to a Ravelin::VectorNd
     * @param v vector
     * @return Ravelin VectorNd
     */
    static VectorNd to_vector(const std::vector<double>& v) {
        return VectorNd(v.size(), &v[0]);
    }

    /**
     * Calculate balancing torques
     * @param req Request
     * @param res Response
     * @return Success
     */
    bool calculateTorques(humanoid_catching::CalculateTorques::Request& req,
                          humanoid_catching::CalculateTorques::Response& res) {

      ROS_INFO("****Calculating torques for arm %s****", req.name.c_str());

      const int num_joints = req.torque_limits.size();

      // Ignore last 3 joints that only impact orientation of end effector
      // that is not used in this formulation
      const int num_effective_joints = num_joints - 3;

      // x_pole
      // linear velocity of body
      ROS_DEBUG("Calculating x_pole vector");
      const Vector3d x_pole = to_vector(req.body_velocity.linear);
      ROS_DEBUG_STREAM("x_pole: " << x_pole);

      // w_pole
      // angular velocity of body
      ROS_DEBUG("Calculating w_pole vector");
      const Vector3d w_pole = to_vector(req.body_velocity.angular);
      ROS_DEBUG_STREAM("w_pole: " << w_pole);

      // v_pole(t)
      // | x_pole |
      // | w_pole |
      ROS_DEBUG("Calculating v_pole vector");
      VectorNd v_pole(POLE_DOF);
      v_pole.set_sub_vec(0, x_pole);
      v_pole.set_sub_vec(3, w_pole);
      ROS_INFO_STREAM("v_pole (" << v_pole.norm() << "): " << v_pole);

      // v_robot
      // joint velocities of the robot
      ROS_DEBUG("Calculating v_robot vector");
      VectorNd v_robot = to_vector(req.joint_velocity);
      v_robot.resize(num_effective_joints);
      ROS_INFO_STREAM("v_robot (" << v_robot.norm() << "): " << v_robot);

      // v(t)
      // | v_pole |
      // | v_robot |
      ROS_DEBUG("Calculating v vector");
      VectorNd v(POLE_DOF + num_effective_joints);
      v.set_sub_vec(0, v_pole);
      v.set_sub_vec(POLE_DOF, v_robot);
      ROS_DEBUG_STREAM("v: " << v);

      // R
      // Pole rotation matrix
      ROS_DEBUG("Calculating R matrix");
      const Matrix3d R = Quatd(req.body_com.orientation.x, req.body_com.orientation.y, req.body_com.orientation.z, req.body_com.orientation.w);
      ROS_DEBUG_STREAM("R: " << R);

      // J
      // Pole inertia matrix
      ROS_DEBUG("Calculating J matrix");
      const Matrix3d J = Matrix3d(&req.body_inertia_matrix[0]);
      ROS_DEBUG_STREAM("J: " << J);

      // RJR_t
      ROS_DEBUG("Calculating RJR_t");
      Matrix3d RJ, RJR_t;
      R.mult(J, RJ);
      RJ.mult_transpose(R, RJR_t);
      ROS_DEBUG_STREAM("RJR_t: " << RJR_t);

      // M_pole
      // | Im   0 |
      // | 0 RJR_t|
      ROS_DEBUG("Calculating M_pole");
      MatrixNd M_pole(POLE_DOF, POLE_DOF);
      M_pole.set_zero();
      M_pole.set_sub_mat(0, 0, Matrix3d::identity() * req.body_mass);
      M_pole.set_sub_mat(3, 3, RJR_t);
      ROS_DEBUG_STREAM("M_pole: " << M_pole);

      // M_robot
      ROS_DEBUG("Calculating M_robot");
      MatrixNd M_robot_base(num_joints, num_joints, &req.robot_inertia_matrix[0]);
      MatrixNd M_robot;
      M_robot_base.get_sub_mat(0, num_effective_joints, 0, num_effective_joints, M_robot);
      ROS_DEBUG_STREAM("M_robot: " << endl << M_robot);

      // M
      // | M_pole 0  |
      // | 0 M_robot |
      ROS_DEBUG("Calculating M");
      MatrixNd M(POLE_DOF + num_effective_joints, POLE_DOF + num_effective_joints);
      M.set_zero();
      M.set_sub_mat(0, 0, M_pole);
      M.set_sub_mat(M_pole.rows(), M_pole.columns(), M_robot);
      ROS_DEBUG_STREAM("M: " << M);

      // J_robot
      // Robot end effector jacobian matrix
      ROS_DEBUG("Calculating J_robot");
      MatrixNd J_robot_base = MatrixNd(6, num_joints, &req.jacobian_matrix[0]);
      MatrixNd J_robot;
      J_robot_base.get_sub_mat(0, 6, 0, num_effective_joints, J_robot);
      ROS_DEBUG_STREAM("J_robot: " << endl << J_robot);

      // delta t
      double delta_t = req.time_delta.toSec();
      ROS_DEBUG_STREAM("delta_t: " << delta_t);

      // Working vector
      Vector3d temp_vector;

      // f_ext
      // | -mg           |
      // | -w x RJR_tw |
      // | 0           |
      ROS_DEBUG("Calculating f_ext");
      VectorNd f_ext(POLE_DOF + num_effective_joints);
      f_ext.set_zero();
      f_ext[2] = req.body_mass * GRAVITY;
      f_ext.set_sub_vec(3, Vector3d::cross(-w_pole, RJR_t.mult(w_pole, temp_vector)));
      ROS_DEBUG_STREAM("f_ext: " << f_ext);

      // n_hat
      // contact normal
      ROS_DEBUG("Calculating n_hat");
      Vector3d n_hat;
      n_hat[0] = 0;
      n_hat[1] = 0;
      n_hat[2] = 1;
      ROS_DEBUG_STREAM("n_hat: " << n_hat);

      // s_hat
      // vector orthogonal to contact normal
      ROS_DEBUG("Calculating s_hat");
      Vector3d s_hat;
      s_hat[0] = 1;
      s_hat[1] = 0;
      s_hat[2] = 0;
      ROS_DEBUG_STREAM("s_hat: " << s_hat);

      // t_hat
      // vector orthogonal to contact normal
      ROS_DEBUG("Calculating t_hat");
      Vector3d t_hat;
      t_hat[0] = 0;
      t_hat[1] = 1;
      t_hat[2] = 0;
      ROS_DEBUG_STREAM("t_hat: " << t_hat);

      // q_hat
      // contact normal
      ROS_DEBUG("Calculating q_hat");
      Vector3d q_hat = to_vector(req.contact_normal);
      ROS_WARN_STREAM("q_hat: " << q_hat);

      // p
      // contact point
      ROS_DEBUG("Calculating P");
      const Vector3d p = to_vector(req.ground_contact);
      ROS_INFO_STREAM("p: " << p);

      // x_bar
      ROS_DEBUG("Calculating x_bar");
      const Vector3d x_bar = to_vector(req.body_com.position);
      ROS_INFO_STREAM("x_bar: " << x_bar);

      // r
      ROS_DEBUG("Calculating r");
      const Vector3d r = p - x_bar;
      ROS_INFO_STREAM("r: " << r);

      // N
      // | n_hat     |
      // | r x n_hat |
      // | 0         |
      ROS_DEBUG("Calculating N");
      VectorNd N(POLE_DOF + num_effective_joints);
      N.set_zero();
      N.set_sub_vec(0, n_hat);
      N.set_sub_vec(3, r.cross(n_hat, temp_vector));
      ROS_DEBUG_STREAM("N: " << N);

      // S
      // | s_hat     |
      // | r x s_hat |
      // | 0         |
      ROS_DEBUG("Calculating S");
      VectorNd S(POLE_DOF + num_effective_joints);
      S.set_zero();
      S.set_sub_vec(0, s_hat);
      S.set_sub_vec(3, r.cross(s_hat, temp_vector));
      ROS_DEBUG_STREAM("S: " << S);

      // T
      // | t_hat     |
      // | r x t_hat |
      // | 0         |
      ROS_DEBUG("Calculating T");
      VectorNd T(POLE_DOF + num_effective_joints);
      T.set_zero();
      T.set_sub_vec(0, t_hat);
      T.set_sub_vec(3, r.cross(t_hat, temp_vector));
      ROS_DEBUG_STREAM("T: " << T);

      // Q
      // | q_hat        |
      // | r x q_hat    |
      // | | -q_hat | J |
      // | |    0   |   |
      ROS_DEBUG("Calculating Q");
      VectorNd Q(POLE_DOF + num_effective_joints);
      Q.set_zero();
      Q.set_sub_vec(0, q_hat);
      Q.set_sub_vec(3, -r.cross(q_hat, temp_vector));

      MatrixNd J_robot_transpose = J_robot;
      J_robot_transpose.transpose();

      VectorNd q_hat_extended(6);
      q_hat_extended.set_zero();
      q_hat_extended.set_sub_vec(0, -q_hat);
      VectorNd J_q_hat(num_effective_joints);
      J_robot_transpose.mult(q_hat_extended, J_q_hat);
      Q.set_sub_vec(POLE_DOF, J_q_hat);
      ROS_DEBUG_STREAM("Q: " << Q);

      // P
      ROS_DEBUG("Calculating P");
      MatrixNd P(POLE_DOF + num_effective_joints, num_effective_joints);
      P.set_zero();
      P.set_sub_mat(POLE_DOF, 0, MatrixNd::identity(num_effective_joints));
      ROS_DEBUG_STREAM("P: " << endl << P);

      // Result vector
      // Torques, f_n, f_s, f_t, f_robot, v_(t + t_delta)
      int torque_idx = 0;
      int f_n_idx = torque_idx + num_effective_joints;
      int f_s_idx = f_n_idx + 1;
      int f_t_idx = f_s_idx + 1;
      int f_robot_idx = f_t_idx + 1;
      int v_t_delta_pole_idx = f_robot_idx + 1;
      int v_t_delta_robot_idx = v_t_delta_pole_idx + POLE_DOF;
      VectorNd z(num_effective_joints + 1 + 1 + 1 + 1 + POLE_DOF + num_effective_joints);
      z.set_zero();

      // Set up minimization function
      ROS_DEBUG("Calculating H");
      MatrixNd H(z.size(), z.size());
      H.set_zero();
      H.set_sub_mat(v_t_delta_pole_idx, v_t_delta_pole_idx, M_pole);
      ROS_DEBUG_STREAM("H: " << H);

      ROS_DEBUG("Calculating c");
      VectorNd c(z.rows());
      c.set_zero(c.rows());
      ROS_DEBUG_STREAM("c: " << c);

      // Linear equality constraints
      ROS_DEBUG("Calculating A + b");
      MatrixNd A(1 * 2 + POLE_DOF + num_effective_joints, z.size());
      A.set_zero();

      VectorNd b(A.rows());
      b.set_zero();

      unsigned int idx = 0;

      ROS_DEBUG("Setting Sv constraint");
      // Sv(t + t_delta) = 0 (no tangent velocity)
      A.set_sub_mat(idx, v_t_delta_pole_idx, S, eTranspose);
      b[idx] = 0;
      idx += 1;

      ROS_DEBUG("Setting Tv constraint");
      // Tv(t + t_delta) = 0 (no tangent velocity)
      A.set_sub_mat(idx, v_t_delta_pole_idx, T, eTranspose);
      b[idx] = 0;
      idx += 1;

      ROS_DEBUG("Setting velocity constraint");
      // v_(t + t_delta) = v_t + M_inv (N_t * f_n + S_t * f_s + T_t * f_t + delta_t * f_ext + delta_t * P_t * torque + delta_t * Q_t * f_robot)
      // Manipulated to fit constraint form
      // -v_t - M_inv * delta_t * f_ext = M_inv * delta_t * P_t * torque + M_inv * N_t * f_n + M_inv * S_t * f_s + M_inv * T_t * f_t + delta_t * M_inv * Q_t * f_robot + -I * v_(t + t_delta)

      ROS_DEBUG("Inverting M");
      MatrixNd M_inverse = M;
      linAlgd.invert(M_inverse);
      ROS_DEBUG_STREAM("M_inverse: " << M_inverse);

      ROS_DEBUG("Calculating M_inverse P");
      MatrixNd M_inverse_P(M_inverse.rows(), P.columns());
      M_inverse.mult(P, M_inverse_P);
      M_inverse_P *= delta_t;
      ROS_DEBUG_STREAM("M_inverse_P" << M_inverse_P);
      A.set_sub_mat(idx, torque_idx, M_inverse_P);

      ROS_DEBUG("Calculating M_inverse N");
      MatrixNd M_inverse_N(M_inverse.rows(), N.columns());
      M_inverse.mult(N, M_inverse_N);
      A.set_sub_mat(idx, f_n_idx, M_inverse_N);
      ROS_DEBUG_STREAM("M_inverse_N" << M_inverse_N);

      ROS_DEBUG("Calculating M_inverse S");
      MatrixNd M_inverse_S(M_inverse.rows(), S.columns());
      M_inverse.mult(S, M_inverse_S);
      ROS_DEBUG_STREAM("M_inverse_S" << M_inverse_S);
      A.set_sub_mat(idx, f_s_idx, M_inverse_S);

      ROS_DEBUG("Calculating M_inverse T");
      MatrixNd M_inverse_T(M_inverse.rows(), T.columns());
      M_inverse.mult(T, M_inverse_T);
      ROS_DEBUG_STREAM("M_inverse_T" << M_inverse_S);
      A.set_sub_mat(idx, f_t_idx, M_inverse_T);

      ROS_DEBUG("Calculating M_inverse Q");
      MatrixNd M_inverse_Q(M_inverse.rows(), Q.columns());
      M_inverse.mult(Q, M_inverse_Q);
      ROS_DEBUG_STREAM("M_inverse_Q" << M_inverse_Q);
      M_inverse_Q *= delta_t;
      A.set_sub_mat(idx, f_robot_idx, M_inverse_Q);
      ROS_DEBUG_STREAM("M_inverse_Q" << M_inverse_Q);

      ROS_DEBUG("Setting Identity Matrix for v_delta_t");
      A.set_sub_mat(idx, v_t_delta_pole_idx, MatrixNd::identity(POLE_DOF + num_effective_joints).negate());

      ROS_DEBUG("Calculating M_inverse F_ext");
      VectorNd M_inverse_F_ext(POLE_DOF + num_effective_joints);
      M_inverse_F_ext.set_zero();
      M_inverse.mult(f_ext, M_inverse_F_ext, delta_t);
      ROS_DEBUG_STREAM("M_inverse F_ext" << M_inverse_F_ext);
      M_inverse_F_ext.negate() -= v;
      ROS_DEBUG_STREAM("-v - M_inverse F_ext" << M_inverse_F_ext);
      b.set_sub_vec(idx, M_inverse_F_ext);

      ROS_DEBUG_STREAM("A: " << endl << A);
      ROS_DEBUG_STREAM("b: " << endl << b);

      // Linear inequality constraints
      ROS_DEBUG("Calculating Mc and q");
      MatrixNd Mc(2, z.size());
      Mc.set_zero();

      VectorNd q(Mc.rows());
      q.set_zero();
      idx = 0;

      // Nv(t) >= 0 (non-negative normal velocity)
      Mc.set_sub_mat(idx, v_t_delta_pole_idx, N, eTranspose);
      q[idx] = 0;
      idx += 1;

      // Qv(t) >= 0 (no interpenetration)
      Mc.set_sub_mat(idx, v_t_delta_pole_idx, Q, eTranspose);
      q[idx] = -EPSILON;
      idx += 1;

      ROS_DEBUG_STREAM("Mc: " << Mc);
      ROS_DEBUG_STREAM("q: " << q);

      // Solution variable constraint
      ROS_DEBUG("Calculating lb and ub");
      VectorNd lb(z.size());
      VectorNd ub(z.size());

      // Torque constraints
      unsigned int bound = 0;
      for (bound; bound < num_effective_joints; ++bound) {
        lb[bound] = req.torque_limits[bound].minimum;
        ub[bound] = req.torque_limits[bound].maximum;
        z[bound] = (req.torque_limits[bound].maximum - req.torque_limits[bound].minimum) / 2.0;
      }

      // f_n >= 0
      lb[bound] = 0;
      ub[bound] = INFINITY;
      z[bound] = 0;
      ++bound;

      // f_s (no constraints)
      lb[bound] = -INFINITY;
      ub[bound] = INFINITY;
      z[bound] = 0;
      ++bound;

      // f_t (no constraints)
      lb[bound] = -INFINITY;
      ub[bound] = INFINITY;
      z[bound] = 0;
      ++bound;

      // f_robot >= 0
      lb[bound] = 0;
      ub[bound] = INFINITY;
      z[bound] = 0;
      ++bound;

      // v_t pole(no constraints)
      for (bound; bound < v_t_delta_robot_idx; ++bound) {
        lb[bound] = -INFINITY;
        ub[bound] = INFINITY;
        z[bound] = 0;
      }

      // v_t robot
      for (unsigned int i = 0; bound < z.size(); ++bound, ++i) {
        // lb[bound] = req.velocity_limits[i].minimum;
        // ub[bound] = req.velocity_limits[i].maximum;
        lb[bound] = -INFINITY;
        ub[bound] = INFINITY;
        z[bound] = 0;
      }

      ROS_DEBUG_STREAM("lb: " << lb);
      ROS_DEBUG_STREAM("ub: " << ub);

      // Call solver
      ROS_DEBUG_STREAM("Calling solver: " << z.size() << " variables, " << Mc.rows() << " inequality constraints and " << A.rows() << " equality constraints");
      Moby::QPOASES qp;
      if (!qp.qp_activeset(H, c, lb, ub, Mc, q, A, b, z)){
        ROS_ERROR("QP failed to find feasible point");
        return false;
      }

      ROS_DEBUG_STREAM("QP solved successfully: " << z);

      VectorNd pole_velocities;
      z.get_sub_vec(v_t_delta_pole_idx, v_t_delta_pole_idx + POLE_DOF, pole_velocities);
      ROS_INFO_STREAM("pole_velocities (" << pole_velocities.norm() << "): " << pole_velocities);

      VectorNd arm_velocities;
      z.get_sub_vec(v_t_delta_robot_idx, v_t_delta_robot_idx + num_effective_joints, arm_velocities);
      ROS_INFO_STREAM("arm_velocities (" << arm_velocities.norm() << "): " << arm_velocities);

      // Check torques
      bound = 0;
      for (bound; bound < num_effective_joints; ++bound) {
        assert(z[bound] >= req.torque_limits[bound].minimum);
        assert(z[bound] <= req.torque_limits[bound].maximum);
      }

      // Check interpenetration constraint
      VectorNd all_velocities(POLE_DOF + num_effective_joints);
      all_velocities.set_sub_vec(0, pole_velocities);
      all_velocities.set_sub_vec(POLE_DOF, arm_velocities);
      assert(VectorNd::dot(all_velocities, Q) >= -EPSILON * 1.01 /* due to floating point issues */);

      double f_robot = z[f_robot_idx];
      ROS_INFO_STREAM("f_robot: " << f_robot << " f_n: " << z[f_n_idx] << " f_s: " << z[f_s_idx] << " f_t: " << z[f_t_idx]);

      VectorNd torques;
      z.get_sub_vec(torque_idx, torque_idx + num_effective_joints, torques);
      ROS_DEBUG_STREAM("Torques: " << torques);

      //
      // Second optimization for robot velocity
      //

      // Result vector
      // Torques, f_n, f_s, f_t, f_robot, v_(t + t_delta)
      torque_idx = 0;
      v_t_delta_robot_idx = num_effective_joints;
      z.resize(num_effective_joints + num_effective_joints);
      z.set_zero();

      // Set up minimization function
      ROS_DEBUG_STREAM("Calculating H");
      H.resize(z.size(), z.size());
      H.set_zero();
      H.set_sub_mat(v_t_delta_robot_idx, v_t_delta_robot_idx, M_robot);
      ROS_DEBUG_STREAM("H: " << H);

      ROS_DEBUG_STREAM("Calculating c");
      c.resize(z.rows());
      c.set_zero(c.rows());
      ROS_DEBUG_STREAM("c: " << c);

      // Linear equality constraints
      ROS_DEBUG_STREAM("Calculating A + b");
      A.resize(num_effective_joints, z.size());
      A.set_zero();

      b.resize(A.rows());
      b.set_zero();

      idx = 0;

      ROS_DEBUG_STREAM("Setting velocity constraint");

      // v_robot(t + t_delta) = v_robot_t + M_inv (delta_t * P_t * torque + delta_t * Q_t * f_robot)

      // Manipulated to fit constraint form
      // -v_robot_t - delta_t * M_inv * Q_t * f_robot = M_inv * delta_t * P_t * torque + -I * v_robot(t + t_delta)

      ROS_DEBUG_STREAM("Inverting M");
      M_inverse = M_robot;
      linAlgd.invert(M_inverse);
      ROS_DEBUG_STREAM("M_inverse: " << endl << M_inverse);

      ROS_DEBUG_STREAM("Calculating M_inverse P");
      // P is the identity matrix
      M_inverse_P = M_inverse;
      M_inverse_P *= delta_t;
      ROS_DEBUG_STREAM("M_inverse_P" << M_inverse_P);
      A.set_sub_mat(idx, torque_idx, M_inverse_P);
      A.set_sub_mat(idx, v_t_delta_robot_idx, MatrixNd::identity(num_effective_joints).negate());

      ROS_DEBUG_STREAM("Calculating M_inverse_Q_f_robot");
      VectorNd M_inverse_Q_f_robot(req.joint_velocity.size());
      M_inverse_Q_f_robot.set_zero();
      VectorNd temp_vector_n;
      M_inverse.mult(Q.get_sub_vec(POLE_DOF, Q.rows(), temp_vector_n), M_inverse_Q_f_robot, delta_t * f_robot);
      ROS_DEBUG_STREAM("M_inverse_Q_f_robot" << M_inverse_Q_f_robot);
      M_inverse_Q_f_robot.negate() -= v_robot;
      ROS_DEBUG_STREAM("-v - M_inverse_Q_f_robot" << M_inverse_Q_f_robot);
      b.set_sub_vec(idx, M_inverse_Q_f_robot);

      ROS_DEBUG_STREAM("A: " << endl << A);
      ROS_DEBUG_STREAM("b: " << endl << b);

      // Linear inequality constraints
      ROS_DEBUG_STREAM("Calculating Mc and q");
      Mc.resize(1, z.size());
      Mc.set_zero();

      q.resize(Mc.rows());
      q.set_zero();
      idx = 0;

      // Qv(t) >= -epsilon (no interpenetration)
      // Q_lower * v_robot >= -Q_upper * v_pole - epsilon
      Mc.set_sub_mat(idx, v_t_delta_robot_idx, Q.get_sub_vec(POLE_DOF, POLE_DOF + num_effective_joints, temp_vector_n), eTranspose);
      q[idx] = -VectorNd::dot(Q.get_sub_vec(0, POLE_DOF, temp_vector_n), pole_velocities) - EPSILON;
      idx += 1;

      ROS_DEBUG_STREAM("Mc: " << Mc);
      ROS_DEBUG_STREAM("q: " << q);

      // Solution variable constraint
      ROS_DEBUG("Calculating lb and ub");
      lb.resize(z.size());
      ub.resize(z.size());

      // Torque constraints
      bound = 0;
      for (bound; bound < num_effective_joints; ++bound) {
        lb[bound] = req.torque_limits[bound].minimum;
        ub[bound] = req.torque_limits[bound].maximum;
        z[bound] = (req.torque_limits[bound].maximum - req.torque_limits[bound].minimum) / 2.0;
      }

      // v_t robot
      for (unsigned int i = 0; bound < z.size(); ++bound, ++i) {
        lb[bound] = -INFINITY;
        ub[bound] = INFINITY;
        // lb[bound] = req.velocity_limits[i].minimum;
        // ub[bound] = req.velocity_limits[i].maximum;
        z[bound] = 0;
      }

      ROS_DEBUG_STREAM("lb: " << lb);
      ROS_DEBUG_STREAM("ub: " << ub);

      // Call solver
      ROS_DEBUG_STREAM("Calling solver: " << z.size() << " variables, " << Mc.rows() << " inequality constraints and " << A.rows() << " equality constraints");
      if (!qp.qp_activeset(H, c, lb, ub, Mc, q, A, b, z)){
        ROS_ERROR("QP failed to find feasible point");
        return false;
      }

      ROS_DEBUG_STREAM("QP solved successfully");

      // Check torques
      bound = 0;
      for (bound; bound < num_effective_joints; ++bound) {
        assert(z[bound] >= req.torque_limits[bound].minimum);
        assert(z[bound] <= req.torque_limits[bound].maximum);
      }

      z.get_sub_vec(v_t_delta_robot_idx, v_t_delta_robot_idx + num_effective_joints, arm_velocities);
      ROS_INFO_STREAM("arm_velocities (" << arm_velocities.norm() << "): " << arm_velocities);

      // Check interpenetration constraint
      all_velocities.set_sub_vec(0, pole_velocities);
      all_velocities.set_sub_vec(POLE_DOF, arm_velocities);
      assert(VectorNd::dot(all_velocities, Q) >= -EPSILON * 1.01 /* due to floating point issues */);

      // Copy over result
      z.get_sub_vec(torque_idx, torque_idx + num_effective_joints, torques);
      ROS_INFO_STREAM("Torques: " << torques);

      // Show robot velocity in robot frame
      VectorNd ee_velocity(6);
      J_robot.mult(arm_velocities, ee_velocity);
      ROS_INFO_STREAM("ee_velocity: " << ee_velocity);

      // Check forward dynamics
      // TODO: Move to function
      /*
      N *= z[f_n_idx];
      ROS_INFO_STREAM("N: " << N);

      S *= z[f_s_idx];
      ROS_INFO_STREAM("S: " << S);

      T *= z[f_t_idx];
      ROS_INFO_STREAM("T: " << T);

      f_ext *= delta_t;
      ROS_INFO_STREAM("f_ext: " << f_ext);

      Q *= f_robot * delta_t;
      ROS_INFO_STREAM("Q: " << Q);

      VectorNd applied_torques;
      P.mult(torques, applied_torques);
      P *= delta_t;
      ROS_INFO_STREAM("Applied torques" << applied_torques);

      VectorNd sum(10);
      sum.set_zero();
      sum += N;
      sum += S;
      sum += T;
      sum += f_ext;
      sum += Q;
      sum += applied_torques;
      sum = M_inverse.mult(sum, temp_vector_n);
      sum += v;
      ROS_INFO_STREAM("vt_d: " << sum);
      */

      res.torques.resize(num_joints);
      for (unsigned int i = torque_idx, j = 0; i < torque_idx + num_effective_joints; ++i, ++j) {
        res.torques[j] = z[i];
      }

      ROS_INFO_STREAM("****QP Completed****");
      return true;
    }
  };
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "balancer");
  Balancer bal;
  ros::spin();
}
