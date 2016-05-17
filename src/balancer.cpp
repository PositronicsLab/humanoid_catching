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
      ROS_INFO("Calculating x vector");
      const Vector3d x = toVector(req.body_velocity.linear);
      ROS_INFO_STREAM("x: " << x);
      // w
      // angular velocity of body
      ROS_INFO("Calculating w vector");
      const Vector3d w = toVector(req.body_velocity.angular);
      ROS_INFO_STREAM("w: " << w);

      // v(t)
      // | x |
      // | w |
      ROS_INFO("Calculating v vector");
      VectorNd v(6);
      v.set_sub_vec(0, x);
      v.set_sub_vec(3, w);
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

      // JRobot
      // Robot end effector jacobian matrix
      ROS_INFO("Calculating JRobot");
      const MatrixNd JRobot = MatrixNd(req.torque_limits.size(), 6, &req.jacobian_matrix[0]);
      ROS_INFO_STREAM("JRobot: " << JRobot);

      // RJR_t
      ROS_INFO("Calculating RJR");
      Matrix3d RJR;
      Matrix3d RTranspose = R.transpose(RTranspose);
      RJR = R.mult(J, RJR).mult(RTranspose, RJR);
      ROS_INFO_STREAM("RJR: " << RJR);

      // M
      // | Im   0 |
      // | 0 RJR_t|
      ROS_INFO("Calculating M");
      MatrixNd M(6, 6);
      M.set_zero(M.rows(), M.columns());
      M.set_sub_mat(0, 0, Matrix3d::identity() * req.body_mass);
      M.set_sub_mat(3, 3, RJR);
      ROS_INFO_STREAM("M: " << M);

      // delta t
      double deltaT = req.time_delta.toSec();

      // Working vector
      Vector3d tempVector;

      // fext
      // | g           |
      // | -w x RJR_tw |
      ROS_INFO("Calculating fExt");
      VectorNd fExt(6);
      fExt[0] = 0;
      fExt[1] = 0;
      fExt[2] = GRAVITY;
      fExt.set_sub_vec(3, Vector3d::cross(-w, RJR.mult(w, tempVector)));
      ROS_INFO_STREAM("fExt: " << fExt);

      // n_hat
      // x component of ground contact position
      ROS_INFO("Calculating nHat");
      Vector3d nHat;
      nHat[0] = req.ground_contact.x;
      nHat[1] = 0;
      nHat[2] = 0;
      ROS_INFO_STREAM("nHat: " << nHat);

      // s_hat
      // s component of contact position
      ROS_INFO("Calculating sHat");
      Vector3d sHat;
      nHat[0] = 0;
      nHat[1] = req.ground_contact.y;
      nHat[2] = 0;
      ROS_INFO_STREAM("sHat: " << sHat);

      // t_hat
      // z component of contact position
      ROS_INFO("Calculating tHat");
      Vector3d tHat;
      tHat[0] = 0;
      tHat[1] = 0;
      tHat[2] = req.ground_contact.z;
      ROS_INFO_STREAM("tHat: " << tHat);

      // q_hat
      // contact normal
      ROS_INFO("Calculating qHat");
      const Vector3d qHat = toVector(req.contact_normal);
      ROS_INFO_STREAM("qHat: " << qHat);

      // p
      // contact point
      ROS_INFO("Calculating P");
      const Vector3d p = toVector(req.ground_contact);
      ROS_INFO_STREAM("p: " << p);

      // x_bar
      // pole COM
      ROS_INFO("Calculating xBar");
      const Vector3d xBar = toVector(req.body_com.position);
      ROS_INFO_STREAM("xBar: " << xBar);

      // r
      ROS_INFO("Calculating r");
      const Vector3d r = p - xBar;
      ROS_INFO_STREAM("r: " << r);

      // N
      // | n_hat     |
      // | r x n_hat |
      ROS_INFO("Calculating N");
      VectorNd N(6);
      N.set_sub_vec(0, nHat);
      N.set_sub_vec(3, r.cross(nHat, tempVector));
      ROS_INFO_STREAM("N: " << N);

      // S
      // | s_hat     |
      // | r x s_hat |
      ROS_INFO("Calculating S");
      VectorNd S(6);
      S.set_sub_vec(0, sHat);
      S.set_sub_vec(3, r.cross(sHat, tempVector));
      ROS_INFO_STREAM("S: " << S);

      // T
      // | t_hat     |
      // | r x t_hat |
      ROS_INFO("Calculating T");
      VectorNd T(6);
      T.set_sub_vec(0, tHat);
      T.set_sub_vec(3, r.cross(tHat, tempVector));
      ROS_INFO_STREAM("T: " << T);

      // Q
      ROS_INFO("Calculating Q");
      VectorNd Q(6);
      Q.set_sub_vec(0, qHat);
      Q.set_sub_vec(3, -r.cross(qHat, tempVector));
      ROS_INFO_STREAM("Q: " << Q);

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
      ROS_INFO("Calculating H");
      MatrixNd H(6, z.size());
      H.set_sub_mat(0, vTDeltaIdx, M);
      ROS_INFO_STREAM("H: " << H);

      ROS_INFO("Calculating c");
      VectorNd c(6);
      c.set_zero(c.rows());
      ROS_INFO_STREAM("c: " << c);

      // Linear equality constraints
      ROS_INFO("Calculating A + b");
      MatrixNd A(1 * 2 + req.torque_limits.size() + 6, z.size());
      VectorNd b(1 * 2 + req.torque_limits.size() + 6);

      ROS_INFO("Setting Sv constraint");
      // Sv(t + t_delta) = 0 (no tangent velocity)
      unsigned idx = 0;
      A.set_sub_mat(idx, vTDeltaIdx, S);
      b.set_sub_vec(idx, VectorNd::zero(1));
      idx += 1;

      ROS_INFO("Setting Tv constraint");
      // Tv(t + t_delta) = 0 (no tangent velocity)
      A.set_sub_mat(idx, vTDeltaIdx, T);
      b.set_sub_vec(idx, VectorNd::zero(1));
      idx += 1;

      ROS_INFO("Setting torque constraint");
      // J_robot(transpose) * Q(transpose) * f_robot = torques
      // Transformed to:
      // J_Robot(transpose) * Q(transpose) * f_robot - I * torques = 0
      MatrixNd JQ(req.torque_limits.size(), 1);
      MatrixNd Jt = JRobot;
      Jt.transpose();
      MatrixNd Qt(Q, eTranspose);
      Jt.mult(Qt, JQ);
      A.set_sub_mat(idx, fRobotIdx, JQ);
      A.set_sub_mat(idx, torqueIdx, MatrixNd::identity(req.torque_limits.size()).negate());
      b.set_sub_vec(idx, VectorNd::zero(req.torque_limits.size()));
      idx += req.torque_limits.size();

      ROS_INFO("Setting velocity constraint");
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
      ROS_INFO_STREAM("A: " << A);
      ROS_INFO_STREAM("b: " << b);

      // Linear inequality constraints
      ROS_INFO("Calculating Mc and q");
      MatrixNd Mc(6, z.size());
      VectorNd q(6);

      // Nv(t) >= 0 (non-negative normal velocity)
      Mc.set_sub_mat(0, vTDeltaIdx, N);
      q.set_sub_vec(0, VectorNd::zero(6));
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

      // v_t (no constraints)
      for (bound; bound < z.size(); ++bound) {
        lb[bound] = INFINITY;
        ub[bound] = INFINITY;
      }
      ROS_INFO_STREAM("lb: " << lb);
      ROS_INFO_STREAM("ub: " << ub);

      // Call solver
      ROS_INFO("Calling solver");
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
