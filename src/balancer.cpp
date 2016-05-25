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

static const double GRAVITY = 9.81;
static const double POLE_DOF = 6;

class Balancer {
private:
    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

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

      ROS_INFO("Calculating torques frame %s", req.header.frame_id.c_str());

      // x_pole
      // linear velocity of body
      ROS_INFO("Calculating x_pole vector");
      const Vector3d x_pole = to_vector(req.body_velocity.linear);
      ROS_INFO_STREAM("x_pole: " << x_pole);

      // w_pole
      // angular velocity of body
      ROS_INFO("Calculating w_pole vector");
      const Vector3d w_pole = to_vector(req.body_velocity.angular);
      ROS_INFO_STREAM("w_pole: " << w_pole);

      // v_pole(t)
      // | x_pole |
      // | w_pole |
      ROS_INFO("Calculating v_pole vector");
      VectorNd v_pole(POLE_DOF);
      v_pole.set_sub_vec(0, x_pole);
      v_pole.set_sub_vec(3, w_pole);
      ROS_INFO_STREAM("v_pole: " << v_pole);

      // v_robot
      // joint velocities of the robot
      ROS_INFO("Calculating v_robot vector");
      const VectorNd v_robot = to_vector(req.joint_velocity);
      ROS_INFO_STREAM("v_robot: " << v_robot);

      // v(t)
      // | v_pole |
      // | v_robot |
      ROS_INFO("Calculating v vector");
      VectorNd v(POLE_DOF + req.joint_velocity.size());
      v.set_sub_vec(0, v_pole);
      v.set_sub_vec(POLE_DOF, v_robot);
      ROS_INFO_STREAM("v: " << v);

      // R
      // Pole rotation matrix
      ROS_INFO("Calculating R matrix");
      const Matrix3d R = Quatd(req.body_com.orientation.x, req.body_com.orientation.y, req.body_com.orientation.z, req.body_com.orientation.w);
      ROS_INFO_STREAM("R: " << R);

      // J
      // Pole inertia matrix
      ROS_INFO("Calculating J matrix");
      ROS_INFO_STREAM("BIM: " << req.body_inertia_matrix.size());
      const Matrix3d J = Matrix3d(&req.body_inertia_matrix[0]);
      ROS_INFO_STREAM("J: " << J);

      // RJR_t
      ROS_INFO("Calculating RJR");
      Matrix3d RJ, RJR;
      R.mult(J, RJ);
      RJ.mult_transpose(R, RJR);
      ROS_INFO_STREAM("RJR: " << RJR);

      // M_pole
      // | Im   0 |
      // | 0 RJR_t|
      ROS_INFO("Calculating M_pole");
      MatrixNd M_pole(POLE_DOF, POLE_DOF);
      M_pole.set_zero();
      M_pole.set_sub_mat(0, 0, Matrix3d::identity() * req.body_mass);
      M_pole.set_sub_mat(3, 3, RJR);
      ROS_INFO_STREAM("M_pole: " << M_pole);

      // M_robot
      ROS_INFO("Calculating M_robot");
      MatrixNd M_robot(req.joint_velocity.size(), req.joint_velocity.size(), &req.robot_inertia_matrix[0]);

      // M
      // | M_pole 0  |
      // | 0 M_robot |
      MatrixNd M(POLE_DOF + req.joint_velocity.size(), POLE_DOF + req.joint_velocity.size());
      M.set_zero();
      M.set_sub_mat(0, 0, M_pole);
      M.set_sub_mat(M_pole.rows(), M_pole.columns(), M_robot);

      // J_robot
      // Robot end effector jacobian matrix
      ROS_INFO("Calculating J_robot");
      const MatrixNd J_robot = MatrixNd(6, req.torque_limits.size(), &req.jacobian_matrix[0]);
      ROS_INFO_STREAM("J_robot: " << J_robot);

      // delta t
      double delta_t = req.time_delta.toSec();

      // Working vector
      Vector3d temp_vector;

      // f_ext
      // | g           |
      // | -w x RJR_tw |
      // | 0           |
      ROS_INFO("Calculating f_ext");
      VectorNd f_ext(POLE_DOF + req.joint_velocity.size());
      f_ext.set_zero();
      f_ext[2] = GRAVITY;
      f_ext.set_sub_vec(3, Vector3d::cross(-w_pole, RJR.mult(w_pole, temp_vector)));
      ROS_INFO_STREAM("f_ext: " << f_ext);

      // n_hat
      // contact normal
      // TODO: Pass in ground contact normal
      ROS_INFO("Calculating n_hat");
      Vector3d n_hat;
      n_hat[0] = 1;
      n_hat[1] = 0;
      n_hat[2] = 0;
      ROS_INFO_STREAM("n_hat: " << n_hat);

      // s_hat
      // vector orthogonal to contact normal
      ROS_INFO("Calculating s_hat");
      Vector3d s_hat;
      s_hat[0] = 0;
      s_hat[1] = 1;
      s_hat[2] = 0;
      ROS_INFO_STREAM("s_hat: " << s_hat);

      // t_hat
      // vector orthogonal to contact normal
      ROS_INFO("Calculating t_hat");
      Vector3d t_hat;
      t_hat[0] = 0;
      t_hat[1] = 0;
      t_hat[2] = 1;
      ROS_INFO_STREAM("t_hat: " << t_hat);

      // q_hat
      // contact normal
      ROS_INFO("Calculating q_hat");
      const Vector3d q_hat = to_vector(req.contact_normal);
      ROS_INFO_STREAM("q_hat: " << q_hat);

      // p
      // contact point
      ROS_INFO("Calculating P");
      const Vector3d p = to_vector(req.ground_contact);
      ROS_INFO_STREAM("p: " << p);

      // x_bar
      // pole COM
      ROS_INFO("Calculating x_bar");
      const Vector3d x_bar = to_vector(req.body_com.position);
      ROS_INFO_STREAM("x_bar: " << x_bar);

      // r
      ROS_INFO("Calculating r");
      const Vector3d r = p - x_bar;
      ROS_INFO_STREAM("r: " << r);

      // N
      // | n_hat     |
      // | r x n_hat |
      // | 0         |
      ROS_INFO("Calculating N");
      VectorNd N(POLE_DOF + req.joint_velocity.size());
      N.set_zero();
      N.set_sub_vec(0, n_hat);
      N.set_sub_vec(3, r.cross(n_hat, temp_vector));
      ROS_INFO_STREAM("N: " << N);

      // S
      // | s_hat     |
      // | r x s_hat |
      // | 0         |
      ROS_INFO("Calculating S");
      VectorNd S(POLE_DOF + req.joint_velocity.size());
      S.set_zero();
      S.set_sub_vec(0, s_hat);
      S.set_sub_vec(3, r.cross(s_hat, temp_vector));
      ROS_INFO_STREAM("S: " << S);

      // T
      // | t_hat     |
      // | r x t_hat |
      // | 0         |
      ROS_INFO("Calculating T");
      VectorNd T(POLE_DOF + req.joint_velocity.size());
      T.set_zero();
      T.set_sub_vec(0, t_hat);
      T.set_sub_vec(3, r.cross(t_hat, temp_vector));
      ROS_INFO_STREAM("T: " << T);

      // Q
      // | q_hat        |
      // | r x q_hat    |
      // | | -q_hat | J |
      // | |    0   |   |
      ROS_INFO("Calculating Q");
      VectorNd Q(POLE_DOF + req.joint_velocity.size());
      Q.set_zero();
      Q.set_sub_vec(0, q_hat);
      Q.set_sub_vec(3, r.cross(q_hat, temp_vector));
      VectorNd q_hat_extended(J.columns());
      q_hat_extended.set_zero();
      q_hat_extended.set_sub_vec(0, -q_hat);
      VectorNd J_q_hat(req.joint_velocity.size());
      J.mult(q_hat_extended, J_q_hat);
      Q.set_sub_vec(6, J_q_hat);
      ROS_INFO_STREAM("Q: " << Q);

      // P
      ROS_INFO("Calculating P");
      MatrixNd P(POLE_DOF + req.joint_velocity.size(), req.joint_velocity.size());
      P.set_zero();
      P.set_sub_mat(6, 0, MatrixNd::identity(req.joint_velocity.size()));

      // Result vector
      // Torques, f_n, f_s, f_t, f_robot, v_(t + t_delta)
      const int torque_idx = 0;
      const int f_n_idx = torque_idx + req.torque_limits.size();
      const int f_s_idx = f_n_idx + 1;
      const int f_t_idx = f_s_idx + 1;
      const int f_robot_idx = f_t_idx + 1;
      const int v_t_delta_idx = f_robot_idx + 1;
      VectorNd z(req.torque_limits.size() + 1 + 1 + 1 + 1 + POLE_DOF + req.joint_velocity.size());

      // Set up minimization function
      ROS_INFO("Calculating H");
      MatrixNd H(z.size(), z.size());
      H.set_sub_mat(v_t_delta_idx, v_t_delta_idx, M_pole);
      ROS_INFO_STREAM("H: " << H);

      ROS_INFO("Calculating c");
      VectorNd c(z.rows());
      c.set_zero(c.rows());
      ROS_INFO_STREAM("c: " << c);

      // Linear equality constraints
      ROS_INFO("Calculating A + b");
      MatrixNd A(1 * 2 + POLE_DOF + req.joint_velocity.size(), z.size());
      A.set_zero();

      VectorNd b(A.rows());
      b.set_zero();

      unsigned idx = 0;

      ROS_INFO("Setting Sv constraint");
      // Sv(t + t_delta) = 0 (no tangent velocity)
      A.set_sub_mat(idx, v_t_delta_idx, S);
      b.set_sub_vec(idx, VectorNd::zero(1));
      idx += 1;

      ROS_INFO("Setting Tv constraint");
      // Tv(t + t_delta) = 0 (no tangent velocity)
      A.set_sub_mat(idx, v_t_delta_idx, T);
      b.set_sub_vec(idx, VectorNd::zero(1));
      idx += 1;

      ROS_INFO("Setting velocity constraint");
      // v_(t + t_delta) = v_t + M_inv (N_t * f_n + S_t * f_s + T_t * f_t + delta_t * f_ext + P_t * torque + delta_t * Q_t * f_robot)
      // Manipulated to fit constraint form
      // -v_t - M_inv * delta_t * f_ext = M_inv * P_t * torque + M_inv * N_t * f_n + M_inv * S_t * f_s + M_inv * T_t * f_t + delta_t * M_inv * Q_t * f_robot + -I * v_(t + t_delta)

      ROS_INFO("Inverting M");
      LinAlgd linAlgd;
      MatrixNd M_inverse = M;
      linAlgd.invert(M_inverse);

      ROS_INFO("Calculating M_inverse P");
      MatrixNd M_inverse_P(M_inverse.rows(), P.columns());
      M_inverse.mult(P, M_inverse_P);
      A.set_sub_mat(idx, torque_idx, M_inverse_P);

      ROS_INFO("Calculating M_inverse N");
      MatrixNd M_inverse_N(M_inverse.rows(), N.columns());
      M_inverse.mult(N, M_inverse_N);
      A.set_sub_mat(idx, f_n_idx, M_inverse_N);

      ROS_INFO("Calculating M_inverse S");
      MatrixNd M_inverse_S(M_inverse.rows(), S.columns());
      M_inverse.mult(S, M_inverse_S);
      A.set_sub_mat(idx, f_s_idx, M_inverse_S);

      ROS_INFO("Calculating M_inverse T");
      MatrixNd M_inverse_T(M_inverse.rows(), T.columns());
      M_inverse.mult(T, M_inverse_T);
      A.set_sub_mat(idx, f_t_idx, M_inverse_T);

      ROS_INFO("Calculating M_inverse Q");
      MatrixNd M_inverse_Q(M_inverse.rows(), Q.columns());
      M_inverse.mult(Q, M_inverse_Q);
      M_inverse_Q *= delta_t;
      A.set_sub_mat(idx, f_robot_idx, M_inverse_Q);

      A.set_sub_mat(idx, v_t_delta_idx, MatrixNd::identity(POLE_DOF + req.joint_velocity.size()).negate());

      ROS_INFO("Calculating M_inverse F_ext");
      VectorNd M_inverse_F_ext(POLE_DOF);
      M_inverse_F_ext.set_zero();
      M_inverse.mult(f_ext, M_inverse_F_ext, delta_t);
      M_inverse_F_ext.negate() -= v;
      b.set_sub_vec(idx, M_inverse_F_ext);

      // Linear inequality constraints
      ROS_INFO("Calculating Mc and q");
      MatrixNd Mc(N.rows() + Q.rows(), z.size());
      Mc.set_zero();

      VectorNd q(Mc.rows());
      q.set_zero();
      idx = 0;

      // Nv(t) >= 0 (non-negative normal velocity)
      Mc.set_sub_mat(idx, v_t_delta_idx, N);
      q.set_sub_vec(idx, VectorNd::zero(N.rows()));
      idx += N.rows();

      // Qv(t) >= 0 (no interpenetration)
      Mc.set_sub_mat(idx, v_t_delta_idx, Q);
      q.set_sub_vec(idx, VectorNd::zero(Q.rows()));

      ROS_INFO_STREAM("Mc: " << Mc);
      ROS_INFO_STREAM("q: " << q);

      // Solution variable constraint
      ROS_INFO("Calculating lb and ub");
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
      lb[bound] = INFINITY;
      ub[bound] = INFINITY;
      ++bound;

      // f_t (no constraints)
      lb[bound] = INFINITY;
      ub[bound] = INFINITY;
      ++bound;

      // f_robot >= 0
      lb[bound] = 0;
      ub[bound] = INFINITY;
      ++bound;

      // v_t and v_robot (no constraints)
      for (bound; bound < z.size(); ++bound) {
        lb[bound] = INFINITY;
        ub[bound] = INFINITY;
      }

      ROS_INFO_STREAM("lb: " << lb);
      ROS_INFO_STREAM("ub: " << ub);

      // Call solver
      ROS_INFO("Calling solver");
      Moby::QPOASES qp;
      if (!qp.qp_activeset(H, c, lb, ub, Mc, q, A, b, z)){
            ROS_ERROR("QP failed to find feasible point");
            return false;
      }

      ROS_INFO_STREAM("QP solved successfully: " << z);

      // Copy over result
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
