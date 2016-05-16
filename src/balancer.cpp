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
    static Vector3d toVector(const geometry_msgs::Vector3& v3) {
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
    static Vector3d toVector(const geometry_msgs::Point& p) {
        Vector3d v;
        v[0] = p.x;
        v[1] = p.y;
        v[2] = p.z;
        return v;
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

      // x
      // linear velocity of body
      const Vector3d x = toVector(req.body_velocity.linear);

      // w
      // angular velocity of body
      const Vector3d w = toVector(req.body_velocity.angular);

      // v(t)
      // | x |
      // | w |
      VectorNd v(6);
      v.set_sub_vec(0, x);
      v.set_sub_vec(3, w);

      // R
      // Pole rotation matrix
      const Matrix3d R = MatrixNd(Quatd(req.body_com.orientation.x, req.body_com.orientation.y, req.body_com.orientation.z, req.body_com.orientation.w));

      // J
      // Pole inertia matrix
      const Matrix3d J = MatrixNd(VectorNd(req.body_inertia_matrix.size(), &req.body_inertia_matrix[0]));

      // JRobot
      // Robot end effector jacobian matrix
      const Matrix3d JRobot = MatrixNd(VectorNd(req.jacobian_matrix.size(), &req.jacobian_matrix[0]));

      // RJR_t
      Matrix3d RJR;
      Matrix3d RTranspose = R.transpose(RTranspose);
      RJR = R.mult(J, RJR).mult(RTranspose, RJR);

      // M
      // | Im   0 |
      // | 0 RJR_t|
      MatrixNd M(6, 6);
      M.set_zero(M.rows(), M.columns());
      M.set_sub_mat(0, 0, Matrix3d::identity() * req.body_mass);
      M.set_sub_mat(3, 3, RJR);

      // delta t
      double deltaT = req.time_delta.toSec();

      // Working vector
      Vector3d tempVector;

      // fext
      // | g           |
      // | -w x RJR_tw |
      VectorNd fExt(6);
      fExt[0] = 0;
      fExt[1] = 0;
      fExt[2] = GRAVITY;
      fExt.set_sub_vec(3, Vector3d::cross(-w, RJR.mult(w, tempVector)));

      // n_hat
      // x component of ground contact position
      Vector3d nHat;
      nHat[0] = req.ground_contact.x;
      nHat[1] = 0;
      nHat[2] = 0;

      // s_hat
      // s component of contact position
      Vector3d sHat;
      nHat[0] = 0;
      nHat[1] = req.ground_contact.y;
      nHat[2] = 0;

      // t_hat
      // z component of contact position
      Vector3d tHat;
      tHat[0] = 0;
      tHat[1] = 0;
      tHat[2] = req.ground_contact.z;

      // q_hat
      // contact normal
      const Vector3d qHat = toVector(req.contact_normal);

      // p
      // contact point
      const Vector3d p = toVector(req.ground_contact);

      // x_bar
      // pole COM
      const Vector3d xBar = toVector(req.body_com.position);

      // r
      const Vector3d r = p - xBar;

      // N
      // | n_hat     |
      // | r x n_hat |
      VectorNd N(6);
      N.set_sub_vec(0, nHat);
      N.set_sub_vec(3, r.cross(nHat, tempVector));

      // S
      // | s_hat     |
      // | r x s_hat |
      VectorNd S(6);
      S.set_sub_vec(0, sHat);
      S.set_sub_vec(3, r.cross(sHat, tempVector));

      // T
      // | t_hat     |
      // | r x t_hat |
      VectorNd T(6);
      T.set_sub_vec(0, tHat);
      T.set_sub_vec(3, r.cross(tHat, tempVector));

      // Q
      VectorNd Q(6);
      Q.set_sub_vec(0, qHat);
      Q.set_sub_vec(3, -r.cross(qHat, tempVector));

      // Result vector
      // Torques, f_n, f_s, f_t, f_robot, v_(t + tdelta)
      const int torqueIdx = 0;
      const int fNIdx = req.torque_limits.size();
      const int fSIdx = fNIdx + 1;
      const int fTIdx = fSIdx + 1;
      const int fRobotIdx = fTIdx + 1;
      const int vTDeltaIdx = fRobotIdx + 1;
      VectorNd z(req.torque_limits.size() + 1 + 1 + 1 + 1 + 6);

      // Set up minimization function
      MatrixNd H(6, z.size());
      H.set_sub_mat(0, vTDeltaIdx, M);

      VectorNd c(6);
      c.set_zero(c.rows());

      // Linear equality constraints
      MatrixNd A(1 * 2 + req.torque_limits.size() + 6, z.size());
      VectorNd b(1 * 2 + req.torque_limits.size() + 6);

      // Sv(t + t_delta) = 0 (no tangent velocity)
      unsigned idx = 0;
      A.set_sub_mat(idx, vTDeltaIdx, S);
      b.set_sub_vec(idx, VectorNd::zero(1));
      idx += 1;

      // Tv(t + t_delta) = 0 (no tangent velocity)
      A.set_sub_mat(idx, vTDeltaIdx, T);
      b.set_sub_vec(idx, VectorNd::zero(1));
      idx += 1;

      // J_robot(transpose) * Q(transpose) * f_robot = torques
      // Transformed to:
      // J_Robot(transpose) * Q(transpose) * f_robot - I * torques = 0
      MatrixNd JQ(req.torque_limits.size(), 1);
      MatrixNd Jt(JRobot.columns(), JRobot.rows());
      JRobot.transpose(Jt);
      Jt.mult(MatrixNd(Q, eTranspose), JQ);
      A.set_sub_mat(idx, fRobotIdx, JQ);
      A.set_sub_mat(idx, torqueIdx, MatrixNd::identity(req.torque_limits.size()).negate());
      b.set_sub_vec(idx, VectorNd::zero(req.torque_limits.size()));
      idx += req.torque_limits.size();

      // v_(t + t_delta) = v_t + M_inv (N_t * f_n + S_t * f_s + T_t * f_t + delta_t * f_ext + Q_t * delta_t * f_robot)
      // Manipulated to fit constraint form
      // -v_t - M_inv * delta_t * f_ext = M_inv * N_t * f_n + M_inv * S_t * f_s + M_inv * T_t * f_t + M_inv * Q_t * delta_t * f_robot + -I * v_(t + t_delta)
      LinAlgd linAlgd;
      MatrixNd MInverse = M;
      linAlgd.pseudo_invert(MInverse);

      MatrixNd MInverseN(MInverse.rows(), N.columns());
      MInverse.mult(N, MInverseN);
      A.set_sub_mat(idx, fNIdx, MInverseN);

      MatrixNd MInverseS(MInverse.rows(), S.columns());
      MInverse.mult(S, MInverseS);
      A.set_sub_mat(idx, fSIdx, MInverseS);

      MatrixNd MInverseT(MInverse.rows(), T.columns());
      MInverse.mult(T, MInverseT);
      A.set_sub_mat(idx, fTIdx, MInverseT);

      MatrixNd MInverseQ(MInverse.rows(), Q.columns());
      MInverse.mult(Q, MInverseQ);
      MInverseQ *= deltaT;
      A.set_sub_mat(idx, fRobotIdx, MInverseQ);

      A.set_sub_mat(idx, vTDeltaIdx, MatrixNd::identity(6).negate());

      VectorNd MInverseFExt(6);
      MInverseFExt.set_zero();
      MInverse.mult(fExt, MInverseFExt, deltaT);
      MInverseFExt.negate() -= v;
      b.set_sub_vec(idx, MInverseFExt);

      // Linear inequality constraints
      MatrixNd Mc(6, z.size());
      VectorNd q(6);

      // Nv(t) >= 0 (non-negative normal velocity)
      Mc.set_sub_mat(0, vTDeltaIdx, N);
      q.set_sub_vec(0, VectorNd::zero(6));

      // Solution variable constraint
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

      // v_t (no constraints)
      for (bound; bound < z.size(); ++bound) {
        lb[bound] = INFINITY;
        ub[bound] = INFINITY;
      }

      // Call solver
      Moby::QPOASES qp;
      if (!qp.qp_activeset(H, p, lb, ub, Mc, q, A, b, z)){
            ROS_ERROR("QP failed to find feasible point");
            return false;
      }

      ROS_INFO_STREAM("QP solved successfully: " << z);

      // Copy over result
      res.torques.resize(req.torque_limits.size());
      for (unsigned int i = 0; i < z.size(); ++i) {
        res.torques[i] = z[i];
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
