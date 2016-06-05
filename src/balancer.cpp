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

      ROS_INFO("Calculating torques");

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
      ROS_DEBUG_STREAM("v_pole: " << v_pole);

      // v_robot
      // joint velocities of the robot
      ROS_DEBUG("Calculating v_robot vector");
      const VectorNd v_robot = to_vector(req.joint_velocity);
      ROS_DEBUG_STREAM("v_robot: " << v_robot);

      // v(t)
      // | v_pole |
      // | v_robot |
      ROS_DEBUG("Calculating v vector");
      VectorNd v(POLE_DOF + req.joint_velocity.size());
      v.set_sub_vec(0, v_pole);
      v.set_sub_vec(POLE_DOF, v_robot);
      ROS_INFO_STREAM("v: " << v);

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
      ROS_DEBUG("Calculating RJR");
      Matrix3d RJ, RJR;
      R.mult(J, RJ);
      RJ.mult_transpose(R, RJR);
      ROS_DEBUG_STREAM("RJR: " << RJR);

      // M_pole
      // | Im   0 |
      // | 0 RJR_t|
      ROS_DEBUG("Calculating M_pole");
      MatrixNd M_pole(POLE_DOF, POLE_DOF);
      M_pole.set_zero();
      M_pole.set_sub_mat(0, 0, Matrix3d::identity() * req.body_mass);
      M_pole.set_sub_mat(3, 3, RJR);
      ROS_DEBUG_STREAM("M_pole: " << M_pole);

      // M_robot
      ROS_DEBUG("Calculating M_robot");
      MatrixNd M_robot(req.joint_velocity.size(), req.joint_velocity.size(), &req.robot_inertia_matrix[0]);
      ROS_DEBUG_STREAM("M_robot: " << M_robot);

      // M
      // | M_pole 0  |
      // | 0 M_robot |
      ROS_DEBUG("Calculating M");
      MatrixNd M(POLE_DOF + req.joint_velocity.size(), POLE_DOF + req.joint_velocity.size());
      M.set_zero();
      M.set_sub_mat(0, 0, M_pole);
      M.set_sub_mat(M_pole.rows(), M_pole.columns(), M_robot);
      ROS_DEBUG_STREAM("M: " << M);

      // J_robot
      // Robot end effector jacobian matrix
      ROS_DEBUG("Calculating J_robot");
      MatrixNd J_robot = MatrixNd(6, req.torque_limits.size(), &req.jacobian_matrix[0]);
      ROS_DEBUG_STREAM("J_robot: " << J_robot);

      // delta t
      double delta_t = req.time_delta.toSec();
      ROS_DEBUG_STREAM("delta_t: " << delta_t);

      // Working vector
      Vector3d temp_vector;

      // f_ext
      // | mg           |
      // | -w x RJR_tw |
      // | 0           |
      ROS_DEBUG("Calculating f_ext");
      VectorNd f_ext(POLE_DOF + req.joint_velocity.size());
      f_ext.set_zero();
      f_ext[2] = req.body_mass * GRAVITY;
      f_ext.set_sub_vec(3, Vector3d::cross(-w_pole, RJR.mult(w_pole, temp_vector)));
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
      // Contact normal is robot to pole, but we want the inverse.
      q_hat *= -1;
      ROS_DEBUG_STREAM("q_hat: " << q_hat);

      // p
      // contact point
      ROS_DEBUG("Calculating P");
      const Vector3d p = to_vector(req.ground_contact);
      ROS_DEBUG_STREAM("p: " << p);

      // x_bar
      // pole COM
      ROS_DEBUG("Calculating x_bar");
      const Vector3d x_bar = to_vector(req.body_com.position);
      ROS_DEBUG_STREAM("x_bar: " << x_bar);

      // r
      ROS_DEBUG("Calculating r");
      const Vector3d r = p - x_bar;
      ROS_DEBUG_STREAM("r: " << r);

      // N
      // | n_hat     |
      // | r x n_hat |
      // | 0         |
      ROS_DEBUG("Calculating N");
      VectorNd N(POLE_DOF + req.joint_velocity.size());
      N.set_zero();
      N.set_sub_vec(0, n_hat);
      N.set_sub_vec(3, r.cross(n_hat, temp_vector));
      ROS_DEBUG_STREAM("N: " << N);

      // S
      // | s_hat     |
      // | r x s_hat |
      // | 0         |
      ROS_DEBUG("Calculating S");
      VectorNd S(POLE_DOF + req.joint_velocity.size());
      S.set_zero();
      S.set_sub_vec(0, s_hat);
      S.set_sub_vec(3, r.cross(s_hat, temp_vector));
      ROS_DEBUG_STREAM("S: " << S);

      // T
      // | t_hat     |
      // | r x t_hat |
      // | 0         |
      ROS_DEBUG("Calculating T");
      VectorNd T(POLE_DOF + req.joint_velocity.size());
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
      VectorNd Q(POLE_DOF + req.joint_velocity.size());
      Q.set_zero();
      Q.set_sub_vec(0, q_hat);
      Q.set_sub_vec(3, r.cross(q_hat, temp_vector));

      MatrixNd J_robot_transpose = J_robot;
      J_robot_transpose.transpose();

      VectorNd q_hat_extended(6);
      q_hat_extended.set_zero();
      q_hat_extended.set_sub_vec(0, -q_hat);
      VectorNd J_q_hat(req.joint_velocity.size());
      J_robot_transpose.mult(q_hat_extended, J_q_hat);
      Q.set_sub_vec(POLE_DOF, J_q_hat);
      ROS_DEBUG_STREAM("Q: " << Q);

      // upper_Q
      // | q_hat        |
      // | r x q_hat    |
      // | 0            |
      ROS_DEBUG("Calculating upper Q");
      VectorNd upper_Q(POLE_DOF + req.joint_velocity.size());
      upper_Q.set_zero();
      upper_Q.set_sub_vec(0, q_hat);
      upper_Q.set_sub_vec(3, r.cross(q_hat, temp_vector));

      // P
      ROS_DEBUG("Calculating P");
      MatrixNd P(POLE_DOF + req.joint_velocity.size(), req.joint_velocity.size());
      P.set_zero();
      P.set_sub_mat(POLE_DOF, 0, MatrixNd::identity(req.joint_velocity.size()));
      ROS_DEBUG_STREAM("P: " << endl << P);

      // Result vector
      // Torques, f_n, f_s, f_t, f_robot, v_(t + t_delta)
      const int torque_idx = 0;
      const int f_n_idx = torque_idx + req.torque_limits.size();
      const int f_s_idx = f_n_idx + 1;
      const int f_t_idx = f_s_idx + 1;
      const int f_robot_idx = f_t_idx + 1;
      const int v_t_delta_idx = f_robot_idx + 1;
      const int v_t_delta_robot_idx = v_t_delta_idx + POLE_DOF;
      VectorNd z(req.torque_limits.size() + 1 + 1 + 1 + 1 + POLE_DOF + req.joint_velocity.size());

      // Set up minimization function
      ROS_DEBUG("Calculating H");
      MatrixNd H(z.size(), z.size());
      H.set_zero();
      H.set_sub_mat(v_t_delta_idx, v_t_delta_idx, M_pole);
      ROS_DEBUG_STREAM("H: " << H);

      ROS_DEBUG("Calculating c");
      VectorNd c(z.rows());
      c.set_zero(c.rows());
      ROS_DEBUG_STREAM("c: " << c);

      // Linear equality constraints
      ROS_DEBUG("Calculating A + b");
      MatrixNd A(1 * 3 + POLE_DOF + req.joint_velocity.size(), z.size());
      A.set_zero();

      VectorNd b(A.rows());
      b.set_zero();

      unsigned idx = 0;

      ROS_DEBUG("Setting Sv constraint");
      // Sv(t + t_delta) = 0 (no tangent velocity)
      A.set_sub_mat(idx, v_t_delta_idx, S, eTranspose);
      b[idx] = 0;
      idx += 1;

      ROS_DEBUG("Setting Tv constraint");
      // Tv(t + t_delta) = 0 (no tangent velocity)
      A.set_sub_mat(idx, v_t_delta_idx, T, eTranspose);
      b[idx] = 0;
      idx += 1;


      // Qv(t) = 0 (no interpenetration)
      A.set_sub_mat(idx, v_t_delta_idx, Q, eTranspose);
      b[idx] = 0;
      idx += 1;

      ROS_DEBUG("Setting velocity constraint");
      // v_(t + t_delta) = v_t + M_inv (N_t * f_n + S_t * f_s + T_t * f_t + delta_t * f_ext + delta_t * P_t * torque + delta_t * Q_t * f_robot)
      // Manipulated to fit constraint form
      // -v_t - M_inv * delta_t * f_ext = M_inv * delta_t * P_t * torque + M_inv * N_t * f_n + M_inv * S_t * f_s + M_inv * T_t * f_t + delta_t * M_inv * Q_t * f_robot + -I * v_(t + t_delta)

      ROS_DEBUG("Inverting M");
      MatrixNd M_inverse = M;
      linAlgd.invert(M_inverse);
      ROS_DEBUG_STREAM("M_inverse: " << endl << M_inverse);

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
      MatrixNd M_inverse_Q(M_inverse.rows(), upper_Q.columns());
      M_inverse.mult(upper_Q, M_inverse_Q);
      ROS_DEBUG_STREAM("M_inverse_Q" << M_inverse_Q);
      M_inverse_Q *= delta_t;
      A.set_sub_mat(idx, f_robot_idx, M_inverse_Q);
      ROS_DEBUG_STREAM("M_inverse_Q" << M_inverse_Q);

      ROS_DEBUG("Setting Identity Matrix for v_delta_t");
      A.set_sub_mat(idx, v_t_delta_idx, MatrixNd::identity(POLE_DOF + req.joint_velocity.size()).negate());

      ROS_DEBUG("Calculating M_inverse F_ext");
      VectorNd M_inverse_F_ext(POLE_DOF + req.joint_velocity.size());
      M_inverse_F_ext.set_zero();
      M_inverse.mult(f_ext, M_inverse_F_ext, delta_t);
      ROS_INFO_STREAM("M_inverse F_ext" << M_inverse_F_ext);
      M_inverse_F_ext.negate() -= v;
      ROS_INFO_STREAM("-v - M_inverse F_ext" << M_inverse_F_ext);
      b.set_sub_vec(idx, M_inverse_F_ext);

      ROS_DEBUG_STREAM("A: " << endl << A);
      ROS_DEBUG_STREAM("b: " << endl << b);

      // Linear inequality constraints
      ROS_DEBUG("Calculating Mc and q");
      MatrixNd Mc(1, z.size());
      Mc.set_zero();

      VectorNd q(Mc.rows());
      q.set_zero();
      idx = 0;

      // Nv(t) >= 0 (non-negative normal velocity)
      Mc.set_sub_mat(idx, v_t_delta_idx, N, eTranspose);
      q[idx] = 0;
      idx += 1;

      ROS_DEBUG_STREAM("Mc: " << Mc);
      ROS_DEBUG_STREAM("q: " << q);

      // Solution variable constraint
      ROS_DEBUG("Calculating lb and ub");
      VectorNd lb(z.size());
      VectorNd ub(z.size());

      // Torque constraints
      unsigned int bound = 0;
      for (bound; bound < req.torque_limits.size(); ++bound) {
        lb[bound] = req.torque_limits[bound].minimum;
        ub[bound] = req.torque_limits[bound].maximum;
      }

      // f_n >= 0
      lb[bound] = 0;
      ub[bound] = INFINITY;
      ++bound;

      // f_s (no constraints)
      lb[bound] = -INFINITY;
      ub[bound] = INFINITY;
      ++bound;

      // f_t (no constraints)
      lb[bound] = -INFINITY;
      ub[bound] = INFINITY;
      ++bound;

      // f_robot <= 0
      lb[bound] = -INFINITY;
      ub[bound] = 0;
      ++bound;

      // v_t pole(no constraints)
      for (bound; bound < v_t_delta_robot_idx; ++bound) {
        lb[bound] = -INFINITY;
        ub[bound] = INFINITY;
      }

      // v_t robot
      for (bound; bound < z.size(); ++bound) {
        // Velocity constraints are currently disabled as they cause the QP often to fail
        // to solve.
        // lb[bound] = -INFINITY;
        // ub[bound] = INFINITY;
        lb[bound] = req.velocity_limits[bound - v_t_delta_robot_idx].minimum;
        ub[bound] = req.velocity_limits[bound - v_t_delta_robot_idx].maximum;
      }

      ROS_DEBUG_STREAM("lb: " << lb);
      ROS_DEBUG_STREAM("ub: " << ub);

      // Call solver
      ROS_DEBUG("Calling solver");
      Moby::QPOASES qp;
      if (!qp.qp_activeset(H, c, lb, ub, Mc, q, A, b, z)){
        ROS_ERROR("QP failed to find feasible point");
        return false;
      }

      ROS_INFO_STREAM("QP solved successfully: " << z);

      // Copy over result
      VectorNd torques;
      z.get_sub_vec(torque_idx, torque_idx + req.torque_limits.size(), torques);
      ROS_INFO_STREAM("Torques: " << torques);

      VectorNd pole_velocities;
      z.get_sub_vec(v_t_delta_idx, v_t_delta_idx + POLE_DOF, pole_velocities);
      ROS_INFO_STREAM("pole_velocities: " << pole_velocities);

      VectorNd arm_velocities;
      z.get_sub_vec(v_t_delta_robot_idx, v_t_delta_robot_idx + req.torque_limits.size(), arm_velocities);
      ROS_INFO_STREAM("v_robot: " << v_robot);
      ROS_INFO_STREAM("arm_velocities: " << arm_velocities);

      ROS_INFO_STREAM("f_robot: " << z[f_robot_idx] << " f_n: " << z[f_n_idx] << " f_s: " << z[f_s_idx] << " f_t: " << z[f_t_idx]);

      VectorNd N_f_n = N;
      N_f_n *= z[f_n_idx];
      ROS_INFO_STREAM("Nf_n" << N_f_n);

      VectorNd S_f_s = S;
      S_f_s *= z[f_s_idx];
      ROS_INFO_STREAM("Sf_S" << S_f_s);

      VectorNd T_f_t = T;
      T_f_t *= z[f_t_idx];
      ROS_INFO_STREAM("Tf_t" << T_f_t);

      VectorNd Q_f_q = M_inverse_Q;
      Q_f_q *= z[f_robot_idx] * delta_t;
      ROS_INFO_STREAM("Qf_q" << Q_f_q);

      VectorNd P_torques(POLE_DOF + req.torque_limits.size());
      M_inverse_P.mult(torques, P_torques);
      // THIS SHOULD BE ARM VELOCITY CHANGE
      ROS_INFO_STREAM("MinvP_torques: " << P_torques);

      VectorNd all_velocities;
      z.get_sub_vec(v_t_delta_idx, v_t_delta_idx + POLE_DOF + req.torque_limits.size(), all_velocities);

      double Q_t = Q.dot(all_velocities);
      ROS_INFO_STREAM("Q_t" << Q_t);

      res.torques.reserve(req.torque_limits.size());
      for (unsigned int i = torque_idx; i < torque_idx + req.torque_limits.size(); ++i) {
        res.torques.push_back(z[i]);
      }
      return true;
    }
  };
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "balancer");
  Balancer bal;
  ros::spin();
}
