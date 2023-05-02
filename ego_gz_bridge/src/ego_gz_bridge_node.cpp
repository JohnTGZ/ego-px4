#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <tf2_ros/transform_listener.h>

class EgoGZBridge {
public:

  void init() {
    ROS_INFO("Bridge initialized");

    // Subscribers
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &EgoGZBridge::state_cb, this);
    // TODO Move into a separate callback queue
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &EgoGZBridge::pose_cb, this);

    // Publishers
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // Service clients
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    tfListener.reset(new tf2_ros::TransformListener(tfBuffer));
  }

  void state_cb(const mavros_msgs::State::ConstPtr &msg)
  {
    // ROS_INFO("Vehicle: State: %s, Connected: %i",
    //         current_state.mode.c_str(), current_state.connected);
    current_state = *msg;
  }

  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    // Publish TF from 'base_link' to 'map'
    static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // tf::Transform transform = tf::Transform(tf::Quaternion( msg->pose.orientation.x,
    //                                                 msg->pose.orientation.y,
    //                                                 msg->pose.orientation.z,
    //                                                 msg->pose.orientation.w),
    //                                     tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    tf::Stamped<tf::Pose> map_to_base_link_tf;
    tf::poseStampedMsgToTF(*msg, map_to_base_link_tf);

    br.sendTransform(tf::StampedTransform(map_to_base_link_tf, ros::Time::now(), "map", "base_link"));
  }

  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tfListener;

  mavros_msgs::State current_state;
  // tf::TransformListener tf_listener;

  // Subscribers
  ros::Subscriber state_sub;
  // TODO Move into a separate callback queue
  ros::Subscriber pose_sub;

  // Publishers
  ros::Publisher local_pos_pub;

  // Service clients
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;

private:

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_gz_bridge_node");

  ROS_INFO("Starting up ego_gz_bridge node");

  EgoGZBridge bridge;

  bridge.init();

  // The setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // Wait for FCU Connection
  while (ros::ok() && !bridge.current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0.5;

  // Send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    bridge.local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  // Set to offboard mode
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request_t = ros::Time::now();

  ROS_INFO("Entering main loop");

  while (ros::ok())
  {
    bool request_timeout = (ros::Time::now() - last_request_t > ros::Duration(2.0));
    if (bridge.current_state.mode != "OFFBOARD" && request_timeout)
    {
      ROS_INFO("Setting to offboard mode");
      if (bridge.set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request_t = ros::Time::now();
    }
    else
    {
      if (!bridge.current_state.armed && request_timeout)
      {
        if (bridge.arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request_t = ros::Time::now();
      }
    }

    bridge.local_pos_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}