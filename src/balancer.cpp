#include <ros/ros.h>
#include <humanoid_catching/CalculateTorques.h>
#include <Moby/qpOASES.h>
#include <Ravelin/VectorNd.h>
#include <Ravelin/MatrixNd.h>
#include <Ravelin/Quatd.h>
#include <Ravelin/Opsd.h>
#include <Moby/qpOASES.h>

namespace {
using namespace std;
using namespace humanoid_catching;

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

       balancerService = nh.advertiseService("/balancer/calculate_torques",
            &Balancer::calculateTorques, this);
	}

private:

    static Ravelin::Vector3d toVector(const geometry_msgs::Vector3& v3) {
        Ravelin::Vector3d v;
        v[0] = v3.x;
        v[1] = v3.y;
        v[2] = v3.z;
        return v;
    }

    static Ravelin::Vector3d toVector(const geometry_msgs::Point& p) {
        Ravelin::Vector3d v;
        v[0] = p.x;
        v[1] = p.y;
        v[2] = p.z;
        return v;
    }

    bool calculateTorques(humanoid_catching::CalculateTorques::Request& req,
               humanoid_catching::CalculateTorques::Response& res) {

      ROS_INFO("Calculating torques frame %s", req.header.frame_id.c_str());

      // x
      // linear velocity of body
      Ravelin::Vector3d x = toVector(req.body_velocity.linear);

      // w
      // angular velocity of body
      Ravelin::Vector3d w = toVector(req.body_velocity.angular);

      // v(t)
      // | x |
      // | w |
      Ravelin::VectorNd v(6);
      v.set_sub_vec(0, x);
      v.set_sub_vec(3, w);

      // R
      // Pole rotation matrix
      Ravelin::Matrix3d R = Ravelin::MatrixNd(Ravelin::Quatd(req.body_com.orientation.x, req.body_com.orientation.y, req.body_com.orientation.z, req.body_com.orientation.w));

      // J
      // Pole inertia matrix
      Ravelin::Matrix3d J = Ravelin::MatrixNd(&req.body_inertia_matrix[0]);

      // JRobot
      // Robot jacobian matrix
      Ravelin::Matrix3d JRobot = Ravelin::MatrixNd(&req.jacobian_matrix[0]);

      // 3x3 working matrix
      Ravelin::Matrix3d temp;

      // RJR_t
      Ravelin::Matrix3d RJR = R.mult(J, temp).mult(Ravelin::Matrix3d::transpose(R), temp);

      // M
      // | Im   0 |
      // | 0 RJR_t|
      Ravelin::MatrixNd M(6, 6);
      M.set_zero(M.rows(), M.columns());
      M.set_sub_mat(0, 0, Ravelin::Matrix3d::identity() * req.body_mass);
      M.set_sub_mat(3, 3, RJR);

      // delta t
      double deltaT = req.time_delta.toSec();

      // Working vector
      Ravelin::Vector3d tempVector;

      // fext
      // | g           |
      // | -w x RJR_tw |
      Ravelin::VectorNd fExt(6);
      fExt[0] = 0;
      fExt[1] = 0;
      fExt[2] = GRAVITY;
      fExt.set_sub_vec(3, Ravelin::Vector3d::cross(-w, RJR.mult(w, tempVector)));

      // n_hat
      // x component of ground contact position
      Ravelin::Vector3d nHat;
      nHat[0] = req.ground_contact.x;
      nHat[1] = 0;
      nHat[2] = 0;

      // s_hat
      // s component of contact position
      Ravelin::Vector3d sHat;
      nHat[0] = 0;
      nHat[1] = req.ground_contact.y;
      nHat[2] = 0;

      // t_hat
      // z component of contact position
      Ravelin::Vector3d tHat;
      tHat[0] = 0;
      tHat[1] = 0;
      tHat[2] = req.ground_contact.z;

      // q_hat
      // contact normal
      Ravelin::Vector3d qHat = toVector(req.contact_normal);

      // p
      // contact point
      Ravelin::Vector3d p = toVector(req.ground_contact);

      // x_bar
      // pole COM
      Ravelin::Vector3d xBar = toVector(req.body_com.position);

      // r
      Ravelin::Vector3d r = p - xBar;

      // N
      // | n_hat     |
      // | r x n_hat |
      Ravelin::VectorNd N(6);
      N.set_sub_vec(0, nHat);
      N.set_sub_vec(3, r.cross(nHat, tempVector));

      // S
      // | s_hat     |
      // | r x s_hat |
      Ravelin::VectorNd S(6);
      S.set_sub_vec(0, sHat);
      S.set_sub_vec(3, r.cross(sHat, tempVector));

      // T
      // | t_hat     |
      // | r x t_hat |
      Ravelin::VectorNd T(6);
      T.set_sub_vec(0, tHat);
      T.set_sub_vec(3, r.cross(tHat, tempVector));

      // Q
      Ravelin::VectorNd Q(6);
      Q.set_sub_vec(0, qHat);
      Q.set_sub_vec(3, -r.cross(qHat, tempVector));

      // Result vector
      // Torques, f_n, f_s, f_t, f_robot, v_t
      Ravelin::VectorNd z(req.torque_limits.size() + 1 + 1 + 1 + 1 + 6);

      // Set up minimization function
      Ravelin::MatrixNd H(6, 6);
      Ravelin::VectorNd c(6);
      c.set_zero(c.rows());

      // Linear equality constraints
      Ravelin::MatrixNd A(6 * 2 + req.torque_limits.size(), z.size());
      Ravelin::VectorNd b(6 * 2 + req.torque_limits.size());

      // Sv(t) = 0 (no tangent velocity)
      A.set_sub_mat(0, req.torque_limits.size() + 6 * 4, S);
      b.set_sub_vec(0, Ravelin::VectorNd::zero(6));

      // Tv(t) = 0 (no tangent velocity)
      A.set_sub_mat(6, req.torque_limits.size() + 6 * 4, T);
      b.set_sub_vec(6, Ravelin::VectorNd::zero(6));

      // J_robot(transpose) * Q(transpose) * f_robot = torques
      Ravelin::MatrixNd JQ(req.torque_limits.size(), 1);
      Ravelin::MatrixNd Jt(JRobot.columns(), JRobot.rows());
      JRobot.transpose(Jt);
      Ravelin::MatrixNd::mult(Jt, Ravelin::MatrixNd(Q, Ravelin::eTranspose), JQ);
      A.set_sub_mat(6 * 2, req.torque_limits.size() + 3, JQ);
      A.set_sub_mat(6 * 2, 0, Ravelin::MatrixNd::identity(req.torque_limits.size()).negate());
      b.set_sub_vec(6 * 2, Ravelin::VectorNd::zero(7));

      // Linear inequality constraints
      Ravelin::MatrixNd Mc(6, z.size());
      Ravelin::VectorNd q(6);

      // Nv(t) >= 0 (non-negative normal velocity)
      Mc.set_sub_mat(0, req.torque_limits.size() + 6 * 4, N);
      q.set_sub_vec(0, Ravelin::VectorNd::zero(6));

      // Solution variable constraint
      Ravelin::VectorNd lb(z.size());
      Ravelin::VectorNd ub(z.size());

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
      lb[bound] = INFINITY;
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
