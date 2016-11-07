#include <ros/ros.h>
#include <humanoid_catching/PredictFall.h>
#include <humanoid_catching/FallPoint.h>
#include <visualization_msgs/Marker.h>
#include <ode/ode.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/math/constants/constants.hpp>
#include <tf/tf.h>

namespace
{
using namespace std;
using namespace humanoid_catching;

//! Default weight in kg
static const double MASS_DEFAULT = 0.896;
static const double HEIGHT_DEFAULT = 1.8288;
static const double RADIUS_DEFAULT = 0.03175;

static const int MAX_CONTACTS = 6;
static const unsigned int MAX_CORRECTIONS = 100;

static const double BASE_X_DEFAULT = 0.4;
static const double BASE_Y_DEFAULT = 0;
static const double BASE_Z_DEFAULT = 0;

// Deflate the end effector to fix interpenetration
static const double DEFLATION_FACTOR = 0.01;

static const double PI = boost::math::constants::pi<double>();

struct Model
{
    dBodyID body;  // the dynamics body
    dGeomID geom;  // geometries representing this body
};

struct SimulationState {
    //! Ground plane
    dGeomID ground;

    //! Simulation space
    dSpaceID space;

    //! Ground link for joint
    dBodyID groundLink;

    //! World
    dWorldID world;

    //! Joint group for collisions
    dJointGroupID contactgroup;

    //! Joint group for joint between ground and humanoid
    dJointGroupID groundToBodyJG;

    //! Joint between ground and humanoid
    dJointID groundJoint;

    //! Humanoid
    Model humanoid;

    //! End effectors
    vector<Model> endEffectors;

    //! Contacts with end effectors
    vector<boost::optional<dContactGeom> > eeContacts;

    int whichEndEffector(const dBodyID b1, const dBodyID b2) const
    {
        for (unsigned int i = 0; i < endEffectors.size(); ++i)
        {
            if (endEffectors[i].body == b1 || endEffectors[i].body == b2)
            {
                return i;
            }
        }
        return -1;
    }

    void destroyWorld() {
        dJointGroupDestroy(contactgroup);
        dJointGroupDestroy(groundToBodyJG);
        dSpaceDestroy(space);
        dWorldDestroy(world);
    }

    void initWorld()
    {
        world = dWorldCreate();
        space = dSimpleSpaceCreate(0);
        contactgroup = dJointGroupCreate(0);
        ground = dCreatePlane(space, 0, 0, 1, 0);
        dWorldSetGravity(world, 0, 0, -9.81);
        dWorldSetERP(world, 0.2);
        dWorldSetCFM(world, 1e-5);
        dWorldSetContactSurfaceLayer(world, 0.001);
    }

    void initHumanoid(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& velocity,
                      const geometry_msgs::Twist& acceleration, double humanoidMass, double humanoidRadius, double humanoidHeight)
    {

        // Create the object
        humanoid.body = dBodyCreate(world);

        dBodySetPosition(humanoid.body, pose.position.x, pose.position.y, pose.position.z);
        dBodySetLinearVel(humanoid.body, velocity.linear.x, velocity.linear.y, velocity.linear.z);
        dBodySetAngularVel(humanoid.body, velocity.angular.x, velocity.angular.y, velocity.angular.z);

        const dReal q[] = {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
        dBodySetQuaternion(humanoid.body, q);

        dMass m;

        // Use the inertia matrix for a point mass rotating about the ground.
        // Use a very small sphere to mimic a point mass, which was not working properly.
        dMassSetSphereTotal(&m, humanoidMass, 0.01);
        dBodySetMass(humanoid.body, &m);
        humanoid.geom = dCreateCylinder(space, humanoidRadius, humanoidHeight);
        dGeomSetBody(humanoid.geom, humanoid.body);
    }

    void initGroundJoint(const geometry_msgs::Point& base)
    {
        // Create the ground object
        groundLink = dBodyCreate(world);
        groundToBodyJG = dJointGroupCreate(0);

        dBodySetPosition(groundLink, base.x, base.y, base.z);

        // Set it as unresponsive to forces
        dBodySetKinematic(groundLink);

        groundJoint = dJointCreateBall(world, groundToBodyJG);
        dJointAttach(groundJoint, humanoid.body, groundLink);

        dJointSetBallAnchor(groundJoint, base.x, base.y, base.z);
    }

    Model initEndEffector(const geometry_msgs::Pose& endEffector, const geometry_msgs::Twist& velocity, const double inflationFactor, vector<double> dimensions)
    {
        // Create the object
        Model object;
        object.body = dBodyCreate(world);

        ROS_DEBUG("Adding link @ %f %f %f (%f %f %f %f) with dimensions [%f %f %f]",
                  endEffector.position.x, endEffector.position.y, endEffector.position.z,
                  endEffector.orientation.x, endEffector.orientation.y, endEffector.orientation.z, endEffector.orientation.w,
                  dimensions[0], dimensions[1], dimensions[2]);

        object.geom = dCreateBox(space,
                                 dimensions[0] * (1.0 + inflationFactor),
                                 dimensions[1] * (1.0 + inflationFactor),
                                 dimensions[2] * (1.0 + inflationFactor));
        dGeomSetBody(object.geom, object.body);
        dBodySetPosition(object.body, endEffector.position.x, endEffector.position.y, endEffector.position.z);
        dBodySetLinearVel(object.body, velocity.linear.x, velocity.linear.y, velocity.linear.z);
        dBodySetAngularVel(object.body, velocity.angular.x, velocity.angular.y, velocity.angular.z);

        const dReal q[] = {endEffector.orientation.x, endEffector.orientation.y, endEffector.orientation.z, endEffector.orientation.w};
        dBodySetQuaternion(object.body, q);

        // Set it as unresponsive to forces
        dBodySetKinematic(object.body);

        return object;
    }

    void collisionCallback(dGeomID o1, dGeomID o2)
    {
        // Ignore ground contacts
        if (o1 == ground || o2 == ground)
        {
            ROS_DEBUG("Ignoring ground contact");
            return;
        }

        dContact contact[MAX_CONTACTS];

        dBodyID b1 = dGeomGetBody(o1);
        dBodyID b2 = dGeomGetBody(o2);

        for (int i = 0; i < MAX_CONTACTS; i++)
        {
            contact[i].surface.mode = dContactSoftCFM;
            contact[i].surface.mu = dInfinity;
            contact[i].surface.mu2 = 0;
            contact[i].surface.soft_cfm = 0.01;
        }

        // Always compute contact with the humanoid as object 1, such that the contact normal
        // is towards the humanoid
        dGeomID firstObj = o1 == humanoid.geom ? o1 : o2;
        dGeomID secondObj = o1 == humanoid.geom ? o2 : o1;

        if (int numc = dCollide(firstObj, secondObj, MAX_CONTACTS, &contact[0].geom, sizeof(dContact)))
        {
            for (int i = 0; i < numc; i++)
            {
                dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
                dJointAttach(c, b1, b2);

                // Determine if this contact should be saved
                int which = whichEndEffector(b1, b2);
                if ((b1 == humanoid.body || b2 == humanoid.body) && which != -1)
                {
                    assert(firstObj == humanoid.geom && secondObj == endEffectors[which].geom
                           && firstObj == contact[i].geom.g1 && secondObj == contact[i].geom.g2);

                    // Only save one contact per end effector
                    eeContacts[which] = contact[i].geom;
                }
            }
        }
    }

    static void staticNearCallback(void* data, dGeomID o1, dGeomID o2)
    {
        SimulationState* self = static_cast<SimulationState*>(data);
        self->collisionCallback(o1, o2);
    }

    void simLoop(double stepSize)
    {
        dSpaceCollide(space, this, &SimulationState::staticNearCallback);
        dWorldStep(world, stepSize);
        dJointGroupEmpty(contactgroup);
    }

    void adjustEndEffector(Model ee)
    {

        // Contacts
        dContact contact[MAX_CONTACTS];
        for (int i = 0; i < MAX_CONTACTS; i++)
        {
            contact[i].surface.mode = dContactSoftCFM;
            contact[i].surface.mu = dInfinity;
            contact[i].surface.mu2 = 0;
            contact[i].surface.soft_cfm = 0.01;
        }

        unsigned int i = 0;
        // Correct interpenetration at t=0 by reducing the size of the bounding box of the end effector.
        while(i < MAX_CORRECTIONS && dCollide(humanoid.geom, ee.geom, MAX_CONTACTS, &contact[0].geom, sizeof(dContact)) > 0)
        {
            if (contact[0].geom.depth <= 0.001)
            {
                break;
            }
            dVector3 boxSize;
            dGeomBoxGetLengths(ee.geom, boxSize);
            dGeomBoxSetLengths(ee.geom, boxSize[0] * (1 - DEFLATION_FACTOR),  boxSize[1] * (1 - DEFLATION_FACTOR),  boxSize[2] * (1 - DEFLATION_FACTOR));
            ++i;
        }
    }
};

class FallPredictor
{
private:
    //! Publisher for the fall visualization
    ros::Publisher fallVizPub;

    //! Publisher for the contact visualization
    ros::Publisher contactVizPub;

    //! Publisher for the initial position
    ros::Publisher initialPoseVizPub;

    //! Publisher for the initial velocity
    ros::Publisher initialVelocityVizPub;

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Fall prediction Service
    ros::ServiceServer fallPredictionService;

    //! Height of object
    double humanoidHeight;

    //! Radius of object
    double humanoidRadius;

    //! Mass of the humanoid
    double humanoidMass;

    //! Inflation factor for end-effector
    double inflationFactor;

    //! Position of base
    geometry_msgs::Point base;
public:
    FallPredictor(const string& name) :
        pnh("~")
    {
        pnh.param("humanoid_height", humanoidHeight, HEIGHT_DEFAULT);
        pnh.param("humanoid_radius", humanoidRadius, RADIUS_DEFAULT);
        pnh.param("humanoid_mass", humanoidMass, MASS_DEFAULT);

        pnh.param("base_x", base.x, BASE_X_DEFAULT);
        pnh.param("base_y", base.y, BASE_Y_DEFAULT);
        pnh.param("base_z", base.z, BASE_Z_DEFAULT);
        pnh.param("inflation_factor", inflationFactor, 0.0);

        fallVizPub = nh.advertise<visualization_msgs::Marker>(
                         "/" + name + "/projected_path", 1);

        contactVizPub = nh.advertise<visualization_msgs::Marker>(
                            "/" + name + "/contacts", 1);

        initialPoseVizPub = nh.advertise<geometry_msgs::PoseStamped>(
                                "/" + name + "/initial_pose", 1);

        initialVelocityVizPub = nh.advertise<geometry_msgs::WrenchStamped>(
                                    "/" + name + "/initial_velocity", 1);

        fallPredictionService = nh.advertiseService("/" + name + "/predict_fall",
                                &FallPredictor::predict, this);

        initODE();
    }

    ~FallPredictor() {
        // Clean up
        destroyODE();
    }

private:

    void initODE()
    {
        dInitODE();
    }

    void destroyODE()
    {
        dCloseODE();
    }

    void publishContacts(const vector<humanoid_catching::Contact>& contacts, const string& frame) const
    {

        for (unsigned int i = 0; i < contacts.size(); ++i)
        {
            if (!contacts[i].is_in_contact)
            {
                visualization_msgs::Marker deleteAll;
                deleteAll.ns = "contacts";
                deleteAll.id = i;
                deleteAll.header.frame_id = frame;
                deleteAll.header.stamp = ros::Time::now();
                deleteAll.action = visualization_msgs::Marker::DELETE;
                contactVizPub.publish(deleteAll);
            }
            else
            {
                visualization_msgs::Marker arrow;
                arrow.header.frame_id = frame;
                arrow.header.stamp = ros::Time::now();
                arrow.ns = "contacts";
                arrow.id = i;
                arrow.type = visualization_msgs::Marker::ARROW;
                arrow.pose.orientation = quaternionFromVector(contacts[i].normal);
                arrow.pose.position = contacts[i].position;
                arrow.scale.x = 0.5;
                arrow.scale.y = 0.05;
                arrow.scale.z = 0.05;

                // Contacts are blue or magenta
                arrow.color.b = 1.0f;
                if (i == 1)
                {
                    arrow.color.r = 1.0f;
                }
                arrow.color.a = 1.0;
                contactVizPub.publish(arrow);
            }
        }
    }

    void publishEeViz(const vector<geometry_msgs::Pose>& ees, const vector<vector<double> >& eeSizes, const string& frame) const
    {
        for (unsigned int i = 0; i < ees.size(); ++i)
        {
            visualization_msgs::Marker box;
            box.header.frame_id = frame;
            box.header.stamp = ros::Time::now();
            box.ns = "end_effectors";
            box.id = i;
            box.type = visualization_msgs::Marker::CUBE;
            box.pose = ees[i];
            box.scale.x = eeSizes[i][0];
            box.scale.y = eeSizes[i][1];
            box.scale.z = eeSizes[i][2];

            // Box is red
            box.color.r = 1.0f;
            if (i == 1)
            {
                box.color.b = 1.0f;
            }
            box.color.a = 0.5;
            fallVizPub.publish(box);
        }
    }

    void publishPathViz(const vector<geometry_msgs::Pose>& path, const string& frame) const
    {

        for (unsigned int i = 0; i < path.size(); ++i)
        {
            visualization_msgs::Marker cyl;
            cyl.header.frame_id = frame;
            cyl.header.stamp = ros::Time::now();
            cyl.ns = "pole";
            cyl.id = i;
            cyl.type = visualization_msgs::Marker::CYLINDER;
            cyl.pose.orientation = orientPose(path[i].orientation, false);
            cyl.pose.position = path[i].position;
            cyl.scale.x = humanoidRadius;
            cyl.scale.y = humanoidRadius;
            cyl.scale.z = humanoidHeight;

            // Cylinder is green
            cyl.color.g = 1.0f;
            cyl.color.a = 0.5;
            fallVizPub.publish(cyl);
        }
    }

    void publishPoseAndVelocity(std_msgs::Header& header, geometry_msgs::Pose& pose, geometry_msgs::Twist& velocity)
    {
        if (initialPoseVizPub.getNumSubscribers() > 0)
        {
            geometry_msgs::PoseStamped ps;
            ps.header = header;
            ps.pose = pose;
            initialPoseVizPub.publish(ps);
        }
        if (initialVelocityVizPub.getNumSubscribers() > 0)
        {
            geometry_msgs::WrenchStamped ws;
            ws.header = header;
            ws.wrench.force = velocity.linear;
            ws.wrench.torque = velocity.angular;
            initialVelocityVizPub.publish(ws);
        }
    }

    static geometry_msgs::Quaternion quaternionFromVector(const geometry_msgs::Vector3& input)
    {
        tf::Vector3 axisVector(input.x, input.y, input.z);
        tf::Vector3 upVector(0.0, 0.0, 1.0);
        tf::Vector3 rightVector = axisVector.cross(upVector);
        rightVector.normalized();
        tf::Quaternion q(rightVector, -1.0 * acos(axisVector.dot(upVector)));
        q.normalize();
        geometry_msgs::Quaternion orientation;
        tf::quaternionTFToMsg(q, orientation);
        return orientation;
    }

    static geometry_msgs::Vector3 arrayToVector(const dReal* aArray)
    {
        geometry_msgs::Vector3 result;
        result.x = aArray[0];
        result.y = aArray[1];
        result.z = aArray[2];
        return result;
    }

    static geometry_msgs::Point arrayToPoint(const dReal* aArray)
    {
        geometry_msgs::Point result;
        result.x = aArray[0];
        result.y = aArray[1];
        result.z = aArray[2];
        return result;
    }

    static geometry_msgs::Quaternion arrayToQuat(const dReal* aArray)
    {
        geometry_msgs::Quaternion result;
        result.x = aArray[0];
        result.y = aArray[1];
        result.z = aArray[2];
        result.w = aArray[3];
        return result;
    }

    static geometry_msgs::Pose getBodyPose(dBodyID body)
    {
        const dReal* position = dBodyGetPosition(body);
        const dReal* orientation = dBodyGetQuaternion(body);
        geometry_msgs::Pose pose;
        pose.position = arrayToPoint(position);
        pose.orientation = arrayToQuat(orientation);
        return pose;
    }

    static geometry_msgs::Twist getBodyTwist(dBodyID body)
    {
        const dReal* linearVelocity = dBodyGetLinearVel(body);
        const dReal* angularVelocity = dBodyGetAngularVel(body);
        geometry_msgs::Twist twist;
        twist.linear = arrayToVector(linearVelocity);
        twist.angular = arrayToVector(angularVelocity);
        return twist;
    }

    static tf::Vector3 quatToVector(const geometry_msgs::Quaternion& orientationMsg)
    {
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(orientationMsg, orientation);
        tf::Transform rotation(orientation);
        tf::Vector3 xAxis(1, 0, 0);
        tf::Vector3 r = rotation.getBasis() * xAxis;
        return r;
    }

    vector<double> getPoleInertiaMatrix() const
    {
        vector<double> I(9);
        I[0] = I[4] = 1 / 12.0 * humanoidMass * pow(humanoidHeight, 2) + 0.25 * humanoidMass * pow(humanoidRadius, 2);
        I[8] = 0.5 * humanoidMass * pow(humanoidRadius, 2);
        return I;
    }

    /**
     * Given a base position, pole length, and an orientation, compute the COM position
     */
    geometry_msgs::Point computeCOMPosition(const geometry_msgs::Quaternion& orientation) const
    {
        // Rotate the up vector
        tf::Quaternion rotation(orientation.x, orientation.y, orientation.z, orientation.w);

        tf::Vector3 initialVector(1, 0, 0);
        tf::Vector3 rotatedVector = tf::quatRotate(rotation, initialVector);

        // Now set the height
        rotatedVector *= humanoidHeight / 2.0;

        // Now offset by the base
        tf::Vector3 offset(base.x, base.y, base.z);
        rotatedVector += offset;

        geometry_msgs::Point result;
        result.x = rotatedVector.x();
        result.y = rotatedVector.y();
        result.z = rotatedVector.z();
        return result;
    }

    geometry_msgs::Vector3 computeLinearVelocity(const geometry_msgs::Quaternion& orientationMsg,
            const geometry_msgs::Vector3& angular)
    {

        tf::Vector3 r = quatToVector(orientationMsg);

        // Now set the height
        r *= humanoidHeight / 2.0;
        tf::Vector3 w(angular.x, angular.y, angular.z);
        tf::Vector3 linear = -r.cross(w);

        geometry_msgs::Vector3 linearVelocityMsg;
        tf::vector3TFToMsg(linear, linearVelocityMsg);
        return linearVelocityMsg;
    }

    // Compensate for the model being initial aligned to the x axis and oriented
    // up
    static geometry_msgs::Quaternion orientPose(const geometry_msgs::Quaternion& orientationMsg, bool negate)
    {
        tf::Quaternion rotation = tf::createQuaternionFromRPY(0, (negate ? -1.0 : 1.0) * PI / 2.0, 0);

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(orientationMsg, orientation);
        orientation *= rotation;
        geometry_msgs::Quaternion result;
        tf::quaternionTFToMsg(orientation, result);
        return result;
    }

    bool predict(humanoid_catching::PredictFall::Request& req,
                 humanoid_catching::PredictFall::Response& res)
    {

        ROS_INFO("Predicting fall in frame %s for %lu links", req.header.frame_id.c_str(), req.end_effectors.size());

        res.header = req.header;

        SimulationState state;
        state.initWorld();

        ROS_DEBUG("Initializing humanoid with orientation: (%f %f %f %f) and velocity: (%f %f %f)",
                 req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w,
                 req.velocity.angular.x, req.velocity.angular.y, req.velocity.angular.z);

        geometry_msgs::Pose humanoidPose;
        humanoidPose.position = computeCOMPosition(req.orientation);
        res.base = base;

        ROS_DEBUG("Computed CoM position of (%f %f %f)", humanoidPose.position.x, humanoidPose.position.y, humanoidPose.position.z);

        // TODO: Replace this with linear/angular fusing
        req.velocity.linear = computeLinearVelocity(req.orientation, req.velocity.angular);

        humanoidPose.orientation = orientPose(req.orientation, false);
        res.initial.position = humanoidPose.position;
        res.initial.orientation = orientPose(humanoidPose.orientation, true);

        state.initHumanoid(humanoidPose, req.velocity, req.accel, humanoidMass, humanoidRadius, humanoidHeight);

        if (req.visualize)
        {
            publishPoseAndVelocity(req.header, humanoidPose, req.velocity);
        }

        ROS_DEBUG("Recalculated humanoid linear velocity: (%f %f %f)",
                 req.velocity.linear.x, req.velocity.linear.y, req.velocity.linear.z);
        res.initial_velocity = req.velocity;

        state.initGroundJoint(base);

        for (unsigned int i = 0; i < req.end_effectors.size(); ++i)
        {
            Model ee = state.initEndEffector(req.end_effectors[i], req.end_effector_velocities[i], inflationFactor, req.shapes[i].dimensions);
            state.adjustEndEffector(ee);
            state.endEffectors.push_back(ee);
        }

        // Execute the simulation loop up to MAX_DURATION seconds
        ros::Duration lastTime;
        for (double t = 0; t <= req.max_time.toSec(); t += req.step_size.toSec())
        {
            // Clear end effector contacts
            state.eeContacts.clear();
            state.eeContacts.resize(req.end_effectors.size());

            // Step forward
            state.simLoop(req.step_size.toSec());

            geometry_msgs::Pose bodyPose = getBodyPose(state.humanoid.body);

            // Only record results for requested steps
            if (t != 0 && ros::Duration(t) - lastTime < req.result_step_size)
            {
                continue;
            }

            lastTime = ros::Duration(t);
            FallPoint curr;

            // Get the location of the body for the current iteration
            curr.pose = bodyPose;
            curr.pose.orientation = orientPose(curr.pose.orientation, true);
            curr.ground_contact = arrayToPoint(dBodyGetPosition(state.groundLink));
            curr.velocity = getBodyTwist(state.humanoid.body);
            curr.time = ros::Duration(t);

            ROS_DEBUG("Pose @ time %f position (%f %f %f) orientation (%f %f %f %f)",
                      curr.time.toSec(), curr.pose.position.x, curr.pose.position.y, curr.pose.position.z,
                      curr.pose.orientation.x, curr.pose.orientation.y, curr.pose.orientation.z, curr.pose.orientation.w);

            curr.contacts.resize(state.eeContacts.size());
            for (unsigned int i = 0; i < state.eeContacts.size(); ++i)
            {
                curr.contacts[i].is_in_contact = !!state.eeContacts[i];
                if (state.eeContacts[i])
                {
                    curr.contacts[i].position = arrayToPoint(state.eeContacts[i]->pos);
                    curr.contacts[i].normal = arrayToVector(state.eeContacts[i]->normal);
                }
            }
            res.points.push_back(curr);

            // Determine if the pole is on the ground and end the simulation
            // Check if COM is at a position with height approximately equal to the radius
            if (bodyPose.position.z <= humanoidRadius * (1 + 0.05))
            {
                ROS_INFO("Humanoid is on the ground. Ending simulation @ [%f]s.", t);
                break;
            }
        }

        // Set the mass and inertia matrix
        res.body_mass = humanoidMass;
        res.height = humanoidHeight;
        res.radius = humanoidRadius;
        res.inertia_matrix = getPoleInertiaMatrix();

        // Publish the path
        if (req.visualize && fallVizPub.getNumSubscribers() > 0)
        {
            vector<geometry_msgs::Pose> points;
            for (vector<FallPoint>::const_iterator i = res.points.begin(); i != res.points.end(); ++i)
            {
                points.push_back(i->pose);
            }
            publishPathViz(points, res.header.frame_id);
        }

        if (req.visualize && contactVizPub.getNumSubscribers() > 0)
        {
            for (vector<FallPoint>::const_iterator i = res.points.begin(); i != res.points.end(); ++i)
            {
                publishContacts(i->contacts, res.header.frame_id);
            }

            vector<geometry_msgs::Pose> eePoses;
            vector<vector<double> > eeSizes;
            for (vector<Model>::const_iterator i = state.endEffectors.begin(); i != state.endEffectors.end(); ++i)
            {
                eePoses.push_back(getBodyPose(i->body));
                dVector3 boxSize;
                dGeomBoxGetLengths(i->geom, boxSize);
                vector<double> vecBoxSize(3);
                vecBoxSize[0] = boxSize[0];
                vecBoxSize[1] = boxSize[1];
                vecBoxSize[2] = boxSize[2];
                eeSizes.push_back(vecBoxSize);
            }
            publishEeViz(eePoses, eeSizes, res.header.frame_id);
        }

        state.destroyWorld();

        ROS_INFO("Completed fall prediction");
        return true;
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fall_predictor");
    FallPredictor fp(ros::this_node::getName());
    ros::spin();
}
