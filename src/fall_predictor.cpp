#include <ros/ros.h>
#include <humanoid_catching/PredictFall.h>
#include <humanoid_catching/CreateMeshCache.h>
#include <humanoid_catching/FallPoint.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ode/ode.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/math/constants/constants.hpp>
#include <tf/tf.h>
#include <geometric_shapes/shape_operations.h>
#include <map>
#include <iterator>
#include <algorithm>

#define PROFILE 0

#if(PROFILE)
#include <google/profiler.h>
#endif

namespace
{
using namespace std;
using namespace humanoid_catching;


//! Default weight in kg
static const double MASS_DEFAULT = 0.896;
static const double HEIGHT_DEFAULT = 1.757;
static const double RADIUS_DEFAULT = 0.03175;

static const int MAX_CONTACTS = 20;

static const double PI = boost::math::constants::pi<double>();

struct MeshCacheEntry {
    dTriMeshDataID mesh;
    double* vertices;
    dTriIndex* triangles;
};

typedef typename std::map<string, MeshCacheEntry> MeshCache;


struct Model
{
    dBodyID body;  // the dynamics body
    dGeomID geom;  // geometries representing this body
    string name; // name of the body
    string meshResource; // path to the mesh
    geometry_msgs::Vector3 meshScale; // scale of the mesh
};

struct SimulationState {
    //! Simulation time
    double t;

    //! Steps
    unsigned int steps;

    //! Is in contact
    bool isInContact;

    //! Is in contact with end effector
    bool isInContactWithEndEffector;

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

    //! Links (non end-effectors)
    vector<Model> links;

    //! Contacts with end effectors
    vector<boost::optional<dContactGeom> > eeContacts;

    //! Number of contacts with end effector
    vector<unsigned int> eeContactCount;

    SimulationState() {
        isInContact = false;
        isInContactWithEndEffector = false;
        t = 0.0;
        steps = 0;
    }

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

    int whichLink(const dBodyID b1, const dBodyID b2) const
    {
        for (unsigned int i = 0; i < links.size(); ++i)
        {
            if (links[i].body == b1 || links[i].body == b2)
            {
                return i;
            }
        }
        return -1;
    }

    void destroyWorld() {

        for (unsigned int i = 0; i < links.size(); ++i) {
            if(dGeomGetClass(links[i].geom) == dTriMeshClass) {
                // Get the trimesh id.
                dTriMeshDataID trimesh = dGeomTriMeshGetTriMeshDataID(links[i].geom);

                // Destroy the temporal coherence cache
                dGeomTriMeshClearTCCache(links[i].geom);
            }

            // Destroy geometries.
            dGeomDestroy(links[i].geom);
        }

        for (unsigned int i = 0; i < endEffectors.size(); ++i) {
            if(dGeomGetClass(endEffectors[i].geom) == dTriMeshClass) {
                // Get the trimesh id.
                dTriMeshDataID trimesh = dGeomTriMeshGetTriMeshDataID(endEffectors[i].geom);

                // Destroy the temporal coherence cache
                dGeomTriMeshClearTCCache(endEffectors[i].geom);
            }

            // Destroy geometries.
            dGeomDestroy(endEffectors[i].geom);
        }

        dJointGroupDestroy(contactgroup);
        dJointGroupDestroy(groundToBodyJG);
        dSpaceDestroy(space);
        dWorldDestroy(world);
    }

    void initWorld()
    {
        world = dWorldCreate();
        space = dSimpleSpaceCreate(0);
        // TODO: Assess this parameter
        // space = dHashSpaceCreate(0);
        contactgroup = dJointGroupCreate(0);
        ground = dCreatePlane(space, 0, 0, 1, 0);
        dWorldSetGravity(world, 0, 0, -9.81);
        dWorldSetERP(world, 0.2);
        // TODO: Assess this parameter
        dWorldSetCFM(world, 1e-5);
        // TODO: Assess this parameter
        dWorldSetContactSurfaceLayer(world, 0.001);
    }

    void initHumanoid(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& velocity,
                      double humanoidMass, double humanoidRadius, double humanoidHeight)
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
        ROS_DEBUG("Creating ground joint at [%f, %f %f]", base.x, base.y, base.z);

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

    Model initLink(const string& name, const geometry_msgs::Pose& link, const geometry_msgs::Twist& velocity, const Shape& shape, const MeshCache& meshCache) const
    {
        // Create the object
        Model object;
        object.name = name;
        object.body = dBodyCreate(world);

        if (shape.type == Shape::BOX) {
            ROS_DEBUG("Adding box @ %f %f %f (%f %f %f %f) with dimensions [%f %f %f]",
                    link.position.x, link.position.y, link.position.z,
                    link.orientation.x, link.orientation.y, link.orientation.z, link.orientation.w,
                    shape.dimensions[0], shape.dimensions[1], shape.dimensions[2]);

            object.geom = dCreateBox(space, shape.dimensions[0],
                                     shape.dimensions[1],
                                     shape.dimensions[2]);
        }
        else if (shape.type == Shape::CYLINDER) {
            ROS_DEBUG("Adding cylinder @ %f %f %f (%f %f %f %f) with dimensions [%f %f]",
                    link.position.x, link.position.y, link.position.z,
                    link.orientation.x, link.orientation.y, link.orientation.z, link.orientation.w,
                    shape.dimensions[0], shape.dimensions[1]);

            object.geom = dCreateCylinder(space, shape.dimensions[0], shape.dimensions[1]);
        }
        else if (shape.type == Shape::SPHERE) {
            ROS_DEBUG("Adding sphere @ %f %f %f (%f %f %f %f) with dimensions [%f]",
                    link.position.x, link.position.y, link.position.z,
                    link.orientation.x, link.orientation.y, link.orientation.z, link.orientation.w,
                    shape.dimensions[0]);

            object.geom = dCreateSphere(space, shape.dimensions[0]);
        }
        else if (shape.type == Shape::MESH) {
            ROS_DEBUG("Adding mesh @ %f %f %f (%f %f %f %f) with vertices [%lu] and indices [%lu]",
                    link.position.x, link.position.y, link.position.z,
                    link.orientation.x, link.orientation.y, link.orientation.z, link.orientation.w,
                    shape.vertices.size(), shape.triangles.size());

            dTriMeshDataID data = NULL;
            if (meshCache.at(name).mesh) {
                data = meshCache.at(name).mesh;
            }
            else {
                ROS_ERROR("Mesh for link [%s] could not be found in cache", name.c_str());
            }

            object.geom = dCreateTriMesh(space, data, NULL, NULL, NULL);
            object.meshResource = shape.meshResource;
            object.meshScale = shape.meshScale;
        }
        else {
            ROS_ERROR("Unsupported shape type");
        }

        dGeomSetBody(object.geom, object.body);

        dBodySetPosition(object.body, link.position.x, link.position.y, link.position.z);
        dBodySetLinearVel(object.body, velocity.linear.x, velocity.linear.y, velocity.linear.z);
        dBodySetAngularVel(object.body, velocity.angular.x, velocity.angular.y, velocity.angular.z);

        const dReal q[] = {link.orientation.x, link.orientation.y, link.orientation.z, link.orientation.w};
        dBodySetQuaternion(object.body, q);

        // Set it as unresponsive to forces
        dBodySetKinematic(object.body);

        return object;
    }

    void nearCollisionCallback(dGeomID o1, dGeomID o2)
    {
        // Ignore ground contacts
        if (o1 == ground || o2 == ground)
        {
            ROS_DEBUG("Ignoring ground contact");
            return;
        }

        dBodyID b1 = dGeomGetBody(o1);
        dBodyID b2 = dGeomGetBody(o2);

        // Check if either contact is the humanoid
        if (b1 != humanoid.body && b2 != humanoid.body) {
            ROS_DEBUG("Skipping non-humanoid contact");
            return;
        }

        ROS_DEBUG("Possible contact detected at time [%f]", t);
        isInContact = true;

        dContact contact[MAX_CONTACTS];
        for (int i = 0; i < MAX_CONTACTS; i++)
        {
            contact[i].surface.mode = dContactSoftCFM;
            contact[i].surface.mu = dInfinity;
            // TODO: Assess this parameter
            contact[i].surface.mu2 = 0;
            // TODO: Assess this parameter
            contact[i].surface.soft_cfm = 0.01;
        }

        // Always compute contact with the humanoid as object 1, such that the contact normal
        // is towards the humanoid
        dGeomID firstObj = o1 == humanoid.geom ? o1 : o2;
        dGeomID secondObj = o1 == humanoid.geom ? o2 : o1;

        ROS_DEBUG("Computing contact between human and link at time [%f]", t);
        if (int numc = dCollide(firstObj, secondObj, MAX_CONTACTS, &contact[0].geom, sizeof(dContact)))
        {
            for (int i = 0; i < numc; i++)
            {
                dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
                dJointAttach(c, b1, b2);

                // Set that the human is in contact
                int whichEE = whichEndEffector(b1, b2);
                int link = whichLink(b1, b2);

                isInContact = true;

                // Determine if this contact should be saved
                if (whichEE != -1) {
                    ROS_DEBUG("Contact between human and end-effector [%s] at time [%f]. Previous contact count was %u. Contact position [%f, %f, %f] and normal [%f, %f, %f] at depth [%f]",
                             endEffectors[whichEE].name.c_str(), t, eeContactCount[whichEE], contact[i].geom.pos[0], contact[i].geom.pos[1], contact[i].geom.pos[2],
                             contact[i].geom.normal[0], contact[i].geom.normal[1], contact[i].geom.normal[2],
                             contact[i].geom.depth);

                    assert(firstObj == humanoid.geom && secondObj == endEffectors[whichEE].geom
                           && firstObj == contact[i].geom.g1 && secondObj == contact[i].geom.g2);

                    // Average all the contacts for the end effector
                    if (eeContactCount[whichEE] == 0) {
                        eeContacts[whichEE] = contact[i].geom;
                    }
                    else {
                        for (unsigned int j = 0; j < 3; ++j) {
                            eeContacts[whichEE]->pos[j] = (contact[i].geom.pos[j] + (eeContactCount[whichEE] * eeContacts[whichEE]->pos[j])) / double(eeContactCount[whichEE] + 1);
                        }
                        for (unsigned int j = 0; j < 3; ++j) {
                            eeContacts[whichEE]->normal[j] = (contact[i].geom.normal[j] + (eeContactCount[whichEE] * eeContacts[whichEE]->normal[j])) / double(eeContactCount[whichEE] + 1);
                        }
                    }
                    ++eeContactCount[whichEE];
                    isInContactWithEndEffector = true;
                } else if (link != -1) {
                    ROS_DEBUG("Contact between human and link [%s] at time [%f]", links[link].name.c_str(), t);
                }
                else {
                    ROS_WARN("Contact with unknown link at time [%f]", t);
                }
            }
        }
        else {
            ROS_DEBUG("Collided contacts but received zero contacts");
        }
    }

    static void staticNearCallback(void* data, dGeomID o1, dGeomID o2)
    {
        SimulationState* self = static_cast<SimulationState*>(data);
        self->nearCollisionCallback(o1, o2);
    }

    void simLoop(double stepSize)
    {
        dSpaceCollide(space, this, &SimulationState::staticNearCallback);
        dWorldStep(world, stepSize);
        dJointGroupEmpty(contactgroup);
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

    //! Cache init Service
    ros::ServiceServer initCacheService;

    //! Height of object
    double humanoidHeight;

    //! Radius of object
    double humanoidRadius;

    //! Mass of the humanoid
    double humanoidMass;

    //! Position of base
    geometry_msgs::Point base;

    //! Mesh cache
    MeshCache meshCache;

    //! Previous path size. Used for clearing visualization markers
    unsigned int lastPathSize;

    #if(PROFILE)
    //! Gprofile name
    string profileName;
    #endif

public:
    FallPredictor(const string& name) :
        pnh("~")
    {
        pnh.param("humanoid_height", humanoidHeight, HEIGHT_DEFAULT);
        pnh.param("humanoid_radius", humanoidRadius, RADIUS_DEFAULT);
        pnh.param("humanoid_mass", humanoidMass, MASS_DEFAULT);

        fallVizPub = nh.advertise<visualization_msgs::MarkerArray>(
                         "/" + name + "/projected_path", 1);

        contactVizPub = nh.advertise<visualization_msgs::MarkerArray>(
                            "/" + name + "/contacts", 1);

        initialPoseVizPub = nh.advertise<geometry_msgs::PoseStamped>(
                                "/" + name + "/initial_pose", 1);

        initialVelocityVizPub = nh.advertise<geometry_msgs::WrenchStamped>(
                                    "/" + name + "/initial_velocity", 1);

        fallPredictionService = nh.advertiseService("/" + name + "/predict_fall",
                                &FallPredictor::predict, this);

        initCacheService = nh.advertiseService("/" + name + "/create_mesh_cache",
                                &FallPredictor::initCache, this);

        initODE();

        #if(PROFILE)
        profileName = "/tmp/" + ros::this_node::getName() + ".prof";
        ProfilerStart(profileName.c_str());
        #endif

        lastPathSize = 0;
    }

    ~FallPredictor() {
        #if(PROFILE)
        ProfilerStop();
        #endif

        // Clear out the mesh caches
        for (MeshCache::iterator entry = meshCache.begin(); entry != meshCache.end(); ++entry) {
            // Destroy the mesh.
            dGeomTriMeshDataDestroy(entry->second.mesh);
            delete[] entry->second.triangles;
            delete[] entry->second.vertices;
        }

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

    void publishContacts(const vector<humanoid_catching::Contact>& contacts, const std_msgs::Header& header) const
    {
        visualization_msgs::MarkerArray arrows;
        for (unsigned int i = 0; i < contacts.size(); ++i)
        {
            if (!contacts[i].is_in_contact)
            {
                visualization_msgs::Marker deleteContact;
                deleteContact.ns = "contacts";
                deleteContact.id = i;
                deleteContact.header = header;
                deleteContact.action = visualization_msgs::Marker::DELETE;
                arrows.markers.push_back(deleteContact);
            }
            else
            {
                visualization_msgs::Marker arrow;
                arrow.header = header;
                arrow.ns = "contacts";
                arrow.id = i;
                arrow.type = visualization_msgs::Marker::ARROW;
                arrow.points.resize(2);
                arrow.points[0] = contacts[i].position;
                arrow.points[1].x = contacts[i].normal.x;
                arrow.points[1].y = contacts[i].normal.y;
                arrow.points[1].z = contacts[i].normal.z;

                arrow.scale.x = 0.025;
                arrow.scale.y = 0.050;

                arrow.color.r = 1.0;
                arrow.color.a = 1.0;
                arrows.markers.push_back(arrow);
            }
        }
        contactVizPub.publish(arrows);
    }

    visualization_msgs::Marker textMarker(const string& ns, const std_msgs::Header& header, const unsigned int i, const string& name,
                                          const geometry_msgs::Pose& pose) const {
        visualization_msgs::Marker text;
        text.header = header;
        text.ns = ns + "_text";
        text.id = i;

        text.scale.x = 1.0;
        text.scale.y = 1.0;
        text.scale.z = 0.05;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.pose = pose;
        text.text = name;

        // Text is black
        text.color.r = text.color.b = text.color.g = 1.0f;
        text.color.a = 1.0;
        return text;
    }

    visualization_msgs::Marker objectMarker(const string& ns, const std_msgs::Header& header, unsigned int i, const Model& model, std_msgs::ColorRGBA color) const {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = ns;
        marker.id = i;
        marker.pose = getBodyPose(model.body);

        if(dGeomGetClass(model.geom) == dBoxClass) {
            marker.type = visualization_msgs::Marker::CUBE;
            vector<double> objectSize = getGeomBoxSize(model.geom);

            if (objectSize[0] <= 0.0 || objectSize[1] <= 0.0 || objectSize[2] <= 0.0) {
                ROS_DEBUG("Cannot publish zero scale marker");
                objectSize[0] = objectSize[1] = objectSize[2] = 0.1;
            }

            marker.scale.x = objectSize[0];
            marker.scale.y = objectSize[1];
            marker.scale.z = objectSize[2];
        }
        else if(dGeomGetClass(model.geom) == dCylinderClass) {
            marker.type = visualization_msgs::Marker::CYLINDER;
            vector<double> objectSize = getGeomCylinderSize(model.geom);

            if (objectSize[0] <= 0.0 || objectSize[1] <= 0.0) {
                ROS_DEBUG("Cannot publish zero scale marker");
                objectSize[0] = objectSize[1] = 0.1;
            }

            marker.scale.x = 2 * objectSize[0]; // x diameter
            marker.scale.y = 2 * objectSize[0]; // y diameter
            marker.scale.z = objectSize[1]; // height
        }
        else if(dGeomGetClass(model.geom) == dSphereClass) {
            marker.type = visualization_msgs::Marker::SPHERE;
            double radius = getGeomSphereSize(model.geom);

            if (radius <= 0.0) {
                ROS_DEBUG("Cannot publish zero scale marker");
                radius = 0.1;
            }

            marker.scale.x = 2 * radius; // diameter
        }
        else if(dGeomGetClass(model.geom) == dTriMeshClass) {
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.mesh_resource = model.meshResource;
            marker.scale = model.meshScale;
            marker.mesh_use_embedded_materials = true;
        }
        else {
            ROS_ERROR("Unsupported type");
        }

        marker.color = color;
        return marker;
    }

    void publishEeViz(const vector<Model>& endEffectors, const vector<Model>& links, const std_msgs::Header& header) const
    {
        visualization_msgs::MarkerArray markers;
        for (unsigned int i = 0; i < endEffectors.size(); ++i){
            // Object is yellow
            std_msgs::ColorRGBA c;
            c.r = 1.0f;
            c.g = 1.0f;
            c.a = 0.5;
            markers.markers.push_back(objectMarker("end_effectors", header, i, endEffectors[i], c));
            markers.markers.push_back(textMarker("end_effectors", header, i, endEffectors[i].name, getBodyPose(endEffectors[i].body)));
        }

        for (unsigned int i = 0; i < links.size(); ++i){
            // Object is blue
            std_msgs::ColorRGBA c;
            c.b = 1.0f;
            c.a = 0.5;
            markers.markers.push_back(objectMarker("links", header, i, links[i], c));
            markers.markers.push_back(textMarker("links", header, i, links[i].name, getBodyPose(links[i].body)));
        }
        fallVizPub.publish(markers);
    }

    visualization_msgs::Marker poseMarker(unsigned int i, const geometry_msgs::Pose& pose, const std_msgs::Header& header) const {
        visualization_msgs::Marker cyl;
        cyl.header = header;
        cyl.ns = "pole";
        cyl.id = i;
        cyl.type = visualization_msgs::Marker::CYLINDER;
        cyl.pose.orientation = orientPose(pose.orientation, false);
        cyl.pose.position = pose.position;
        cyl.scale.x = humanoidRadius;
        cyl.scale.y = humanoidRadius;
        cyl.scale.z = humanoidHeight;

        // Cylinder is green unless it is the initial pose and then it is yellow
        if (i == 0) {
            cyl.color.r = 1.0f;
            cyl.color.g = 1.0f;
            cyl.color.a = 0.5;
        }
        else {
            cyl.color.g = 1.0f;
            cyl.color.a = 0.5;
        }
        return cyl;
    }

    visualization_msgs::Marker comMarker(unsigned int i, const geometry_msgs::Pose& pose, const std_msgs::Header& header) const {

        visualization_msgs::Marker point;
        point.header = header;
        point.ns = "com";
        point.id = i;
        point.type = visualization_msgs::Marker::SPHERE;
        point.pose.position = pose.position;
        point.color.r = 1.0f;
        point.color.a = 1.0f;
        point.scale.x = point.scale.y = point.scale.z = 0.05;
        return point;
    }

    void publishPathViz(const vector<geometry_msgs::Pose>& path, const std_msgs::Header& header) const
    {
        visualization_msgs::MarkerArray markers;
        unsigned int i;
        for (i = 0; i < path.size(); ++i)
        {
            markers.markers.push_back(poseMarker(i + 1, path[i], header));
            markers.markers.push_back(comMarker(i + 1, path[i], header));
        }

        for (; i < lastPathSize; ++i) {
            visualization_msgs::Marker cyl;
            cyl.header = header;
            cyl.ns = "pole";
            cyl.id = i + 1;
            cyl.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(cyl);

            visualization_msgs::Marker point;
            point.header = header;
            point.ns = "com";
            point.id = i + 1;
            point.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(point);
        }
        fallVizPub.publish(markers);
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
        ROS_DEBUG("Computing COM position using base position: [%f %f %f]", base.x, base.y, base.z);

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
            const geometry_msgs::Vector3& angular) const
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

    static vector<double> getGeomBoxSize(const dGeomID geom) {
        dVector3 boxSize;
        dGeomBoxGetLengths(geom, boxSize);
        vector<double> vecBoxSize(3);
        vecBoxSize[0] = boxSize[0];
        vecBoxSize[1] = boxSize[1];
        vecBoxSize[2] = boxSize[2];
        return vecBoxSize;
    }

    static vector<double> getGeomCylinderSize(const dGeomID geom) {
        dReal radius, length;
        dGeomCylinderGetParams(geom, &radius, &length);
        vector<double> vecCylinderSize(2);
        vecCylinderSize[0] = radius;
        vecCylinderSize[1] = length;
        return vecCylinderSize;
    }

    static double getGeomSphereSize(const dGeomID geom) {
        return dGeomSphereGetRadius(geom);
    }

    bool initCache(humanoid_catching::CreateMeshCache::Request& req,
                   humanoid_catching::CreateMeshCache::Response& res) {

        for (unsigned int i = 0; i < req.shapes.size(); ++i) {
            if (req.shapes[i].type == Shape::MESH) {
                ROS_DEBUG("Adding mesh for shape [%s] with vertices [%lu] and indices [%lu] to cache",
                        req.names[i].c_str(), req.shapes[i].vertices.size(), req.shapes[i].triangles.size());

                MeshCacheEntry entry;
                entry.mesh = dGeomTriMeshDataCreate();

                // Clone the arrays
                entry.vertices = new double[req.shapes[i].vertices.size()];
                std::copy(req.shapes[i].vertices.begin(), req.shapes[i].vertices.end(), entry.vertices);

                entry.triangles = new dTriIndex[req.shapes[i].triangles.size()];
                std::copy(req.shapes[i].triangles.begin(), req.shapes[i].triangles.end(), entry.triangles);

                dGeomTriMeshDataBuildDouble(entry.mesh, entry.vertices, 3 * sizeof(dReal), int(req.shapes[i].vertices.size() / 3),
                                            entry.triangles, int(req.shapes[i].triangles.size()), 3 * sizeof(dTriIndex));
                meshCache[req.names[i]] = entry;
            }
            else {
                ROS_WARN("Unsupported shape type for cache: [%u]", req.shapes[i].type);
            }
        }
        return true;
    }

    static double floatEquals(double first, double second) {
        return fabs(first - second) < 0.001;
    }

    /**
     * Check that the COM remains the correct distance from the base
     */
    void checkCOM(const geometry_msgs::Point& position) const {
        double dSquared = pow((position.x - base.x), 2) + pow((position.y - base.y), 2) + pow((position.z - base.z), 2);
        if (!floatEquals(dSquared, pow(humanoidHeight / 2.0, 2))) {
            ROS_WARN("COM distance is [%f] which is not correct distance from ground joint [%f]",
                     sqrt(dSquared), humanoidHeight / 2.0);
        }
    }

    bool predict(humanoid_catching::PredictFall::Request& req,
                 humanoid_catching::PredictFall::Response& res)
    {
        ROS_DEBUG("Predicting fall in frame %s for %lu end effector links and %lu collision links. Max time [%i], Contact time [%i], Step size [%i], Result Step size [%i]",
                 req.header.frame_id.c_str(), req.end_effectors.size(), req.links.size(), req.max_time.nsec, req.contact_time.nsec, req.step_size.nsec, req.result_step_size.nsec);

        nh.param("base_x", base.x, 0.5);
        nh.param("base_y", base.y, 0.0);
        nh.param("base_z", base.z, 0.0);

        ROS_DEBUG("base x: %f, base y: %f, base z: %f", base.x, base.y, base.z);

        res.header = req.header;

        SimulationState state;
        state.initWorld();

        ROS_INFO("Initializing humanoid with orientation: (%f %f %f %f) and velocity: (%f %f %f)",
                 req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w,
                 req.velocity.angular.x, req.velocity.angular.y, req.velocity.angular.z);

        geometry_msgs::Pose humanoidPose;
        humanoidPose.position = computeCOMPosition(req.orientation);
        checkCOM(humanoidPose.position);

        ROS_DEBUG("Computed CoM position of (%f %f %f)", humanoidPose.position.x, humanoidPose.position.y, humanoidPose.position.z);

        // TODO: Replace this with linear/angular fusing
        // TODO: Investigate this further
        req.velocity.linear = computeLinearVelocity(req.orientation, req.velocity.angular);

        humanoidPose.orientation = orientPose(req.orientation, false);

        state.initHumanoid(humanoidPose, req.velocity, humanoidMass, humanoidRadius, humanoidHeight);
        publishPoseAndVelocity(req.header, humanoidPose, req.velocity);

        ROS_DEBUG("Recalculated humanoid linear velocity: (%f %f %f)",
                 req.velocity.linear.x, req.velocity.linear.y, req.velocity.linear.z);

        state.initGroundJoint(base);

        ROS_DEBUG("Creating links");
        state.endEffectors.resize(req.end_effectors.size());
        for (unsigned int i = 0; i < req.end_effectors.size(); ++i)
        {
            state.endEffectors[i] = state.initLink(req.end_effectors[i].name, req.end_effectors[i].pose.pose, req.end_effectors[i].velocity, req.end_effectors[i].shape, meshCache);
        }

        state.links.resize(req.links.size());
        for (unsigned int i = 0; i < req.links.size(); ++i)
        {
            state.links[i] = state.initLink(req.links[i].name, req.links[i].pose.pose, req.links[i].velocity, req.links[i].shape, meshCache);
        }
        ROS_DEBUG("Completed creating links");

        state.eeContacts.resize(req.end_effectors.size());
        state.eeContactCount.resize(req.end_effectors.size());
        res.ground_contact = arrayToPoint(dBodyGetPosition(state.groundLink));

        // Preallocate space for the results
        res.points.reserve(req.max_time.toSec() / req.step_size.toSec());

        // Execute the simulation loop up to MAX_DURATION seconds
        bool isOnGround = false;
        ros::Duration lastTime = ros::Duration(0);
        for (state.t = req.step_size.toSec(); state.t <= req.max_time.toSec() && (!state.isInContact || state.t <= req.contact_time.toSec()) && !isOnGround; state.t += req.step_size.toSec())
        {
            // Clear end effector contacts
            for (unsigned int i = 0; i < state.eeContacts.size(); ++i) {
                state.eeContacts[i] = boost::optional<dContactGeom>();
                state.eeContactCount[i] = 0;
            }

            // Step forward
            state.simLoop(req.step_size.toSec());
            state.steps++;

            geometry_msgs::Pose bodyPose = getBodyPose(state.humanoid.body);

            checkCOM(bodyPose.position);

            ROS_DEBUG("Computed CoM position of (%f %f %f)", bodyPose.position.x, bodyPose.position.y, bodyPose.position.z);

            // Determine if the pole is on the ground and end the simulation
            // Check if COM is at a position with height approximately equal to the radius
            if (bodyPose.position.z <= humanoidRadius * (1 + 0.05))
            {
                ROS_INFO("Humanoid is on the ground. Ending simulation @ [%f]s.", state.t);
                isOnGround = true;
            }

            // Only record results for requested steps
            if (!isOnGround && !state.isInContact && lastTime != ros::Duration(0) && ros::Duration(state.t) - lastTime < req.result_step_size)
            {
                continue;
            }
            lastTime = ros::Duration(state.t);

            FallPoint curr;

            // Get the location of the body for the current iteration
            curr.pose.position = bodyPose.position;
            curr.pose.orientation = orientPose(bodyPose.orientation, true);
            curr.velocity = getBodyTwist(state.humanoid.body);
            curr.time = ros::Duration(state.t);

            ROS_DEBUG("Recording pose @ time %f position (%f %f %f) orientation (%f %f %f %f)",
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
                    curr.contacts[i].link = req.end_effectors[i];
                    ROS_DEBUG("Setting contact link to %s", req.end_effectors[i].name.c_str());
                }
            }

            res.points.push_back(curr);
        }

        // Set the mass and inertia matrix
        res.body_mass = humanoidMass;
        res.height = humanoidHeight;
        res.radius = humanoidRadius;
        res.inertia_matrix = getPoleInertiaMatrix();

        // Publish the path
        if (fallVizPub.getNumSubscribers() > 0)
        {
            ROS_DEBUG("Publishing path visualization");
            vector<geometry_msgs::Pose> points;
            for (vector<FallPoint>::const_iterator i = res.points.begin(); i != res.points.end(); ++i)
            {
                points.push_back(i->pose);
            }
            publishPathViz(points, res.header);
            lastPathSize = points.size();
        }

        if (contactVizPub.getNumSubscribers() > 0)
        {
            ROS_DEBUG("Publishing contact visualization");
            for (vector<FallPoint>::const_iterator i = res.points.begin(); i != res.points.end(); ++i)
            {
                // Ignore contacts past the contact threshold
                if (i->time > req.contact_time) {
                    // This will clear contacts
                    vector<humanoid_catching::Contact> emptyContacts(i->contacts.size());
                    publishContacts(emptyContacts, res.header);
                    break;
                }

                // Publish contacts from the first set of contacts. This will match what the catching controller
                // uses
                bool isInContact = false;
                for (unsigned int j = 0; j < i->contacts.size(); ++j) {
                    if (i->contacts[j].is_in_contact) {
                        isInContact = true;
                    }
                }

                if (isInContact) {
                    publishContacts(i->contacts, res.header);
                    break;
                }
            }
        }

        if (fallVizPub.getNumSubscribers() > 0){
            ROS_DEBUG("Publishing link visualization");
            publishEeViz(state.endEffectors, state.links, res.header);
        }

        state.destroyWorld();
        ROS_INFO("Completed fall prediction after [%u] steps. Predicted for [%f]s. Recorded [%lu] events. Ended in contact [%u]. Ended in contact with end-effector [%u]", state.steps, state.t - req.step_size.toSec(), res.points.size(), state.isInContact, state.isInContactWithEndEffector);
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
