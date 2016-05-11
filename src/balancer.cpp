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

      // Set up minimization function
      Ravelin::MatrixNd H(6, 6);
      Ravelin::VectorNd c(6);
      c.set_zero(c.rows());

      // Linear equality constraints
      Ravelin::MatrixNd A(3);
      A.set_column(0, N);
      A.set_column(1, S);
      A.set_column(2, T);

      Ravelin::VectorNd b(3);
      b.set_zero(b.rows());

      // Linear inequality constraints
      Ravelin::VectorNd q;

      // Solution variable constraints (torque)
      Ravelin::VectorNd lb(7);

      Ravelin::VectorNd ub(7);

      // Result vector
      // TODO: Make number of torques automatic
      Ravelin::VectorNd z(7);

      // Call solver
      Moby::QPOASES qp;
      if (!qp.qp_activeset(H, p, lb, ub, M, q, A, b, z)){
            ROS_ERROR("QP failed to find feasible point");
            return false;
      }

      ROS_INFO_STREAM("QP solved successfully: " << z);

      // Copy over result
      res.torques.resize(7);
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
