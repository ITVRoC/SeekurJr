#include <ros/ros.h>

#ifdef ADEPT_PKG
#include <Aria.h>
#else
#include <Aria/Aria.h>
#endif

#include <ArCommands.h>
#include "ArCameraMode.h"
#include "pan_tilt_zoom/pan_tilt_goal.h"

/*
MAX_PAN = 180, ///< maximum degrees the unit can pan (clockwise from top)
MIN_PAN = -180, ///< minimum degrees the unit can pan (counterclockwise from top)
MIN_TILT = -30, ///< minimum degrees the unit can tilt
MAX_TILT = 60, ///< maximum degrees the unit can tilt
MIN_ZOOM = 0, ///< minimum value for zoom
MAX_ZOOM = 32767, ///< maximum value for zoom
*/

namespace ptz_commander {

class Node {
public:
    explicit Node(ros::NodeHandle& node_handle, ArRobot& robot);
    ~Node();

    // Service Control
    void connect();

    void disconnect();

    // Service Execution
    void spinCallback(const ros::TimerEvent&);

    // Callback Methods
    void goalCallback(const pan_tilt_zoom::pan_tilt_goal::ConstPtr& ptz_goal);

protected:
    ArCameraMode* m_camera;
    ros::NodeHandle m_node;
    ros::Publisher m_joint_pub;
    ros::Subscriber m_joint_sub;
};

Node::Node(ros::NodeHandle& node_handle, ArRobot& robot)
    : m_camera(NULL)
    , m_node(node_handle)
{
    m_camera = new ArCameraMode(&robot, "cameraPTZcom4", 'c', 'C');
}

Node::~Node()
{
    disconnect();
    delete m_camera;
}

/** Opens the connection to the PTZ and manages subscriptions/publishers */
void Node::connect()
{
    ROS_INFO_STREAM("PTZ now initializing.");
    m_camera->activate();

    // Publishers : Only publish the most recent reading
    // m_joint_pub = m_node.advertise
    //               <sensor_msgs::JointState>("state", 1);
    // TODO: publish the state of the PTZ?

    // Subscribers : Only subscribe to the most recent instructions
    m_joint_sub
        = m_node.subscribe<pan_tilt_zoom::pan_tilt_goal>("ptz_goal", 1, &Node::goalCallback, this);
}

/** Disconnect */
void Node::disconnect()
{
  // do nothing
}

/** Callback for getting new goal for the camera */
void Node::goalCallback(const pan_tilt_zoom::pan_tilt_goal::ConstPtr& msg)
{
    ROS_DEBUG("PTZ command callback.");

    int pan = msg->pan;
    int tilt = msg->tilt;
    int zoom = msg->zoom;

    m_camera->ptzSetGoal(pan, tilt, zoom);
}

/**
 * Executes at the hz interval defined
 */
void Node::spinCallback(const ros::TimerEvent&)
{
  // do nothing
}

} // namespace ptz_commander

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ptz_node");
    ros::NodeHandle n;

    while (ros::ok()) {
        Aria::init();
        ArRobot robot;

        // Start the robot task loop running in a new background thread. The 'true' argument means if it loses
        // connection the task loop stops and the thread exits.
        robot.runAsync(true);    
        robot.com2Bytes(116,10,1); // Sets PTZ power.

        // Connect to PTZ
        ptz_commander::Node ptz_node(n, robot);
        ptz_node.connect();

        // Set up polling callback
        int hz = 5;
        ros::Timer spin_timer
            = n.createTimer(ros::Duration(1 / hz), &ptz_commander::Node::spinCallback, &ptz_node);

        // Spin until there's a problem or we're in shutdown
        ros::spin();
        Aria::exit(0);

        ROS_ERROR("PTZ disconnected, attempting reconnection.");
        ros::Duration(1.0).sleep();
    }

    ROS_ERROR("PTZ disconnected, finishing.");
    return 0;
}