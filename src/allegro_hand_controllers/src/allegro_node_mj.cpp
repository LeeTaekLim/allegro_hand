// Common allegro node code used by any node. Each node that implements an
// AllegroNode must define the computeDesiredTorque() method.
//
// Author: Felix Duvallet <felix.duvallet@epfl.ch>

#include "allegro_node_mj.h"

std::string jointNames[DOF_JOINTS] =
        {
                "joint_0.0", "joint_1.0", "joint_2.0", "joint_3.0",
                "joint_4.0", "joint_5.0", "joint_6.0", "joint_7.0",
                "joint_8.0", "joint_9.0", "joint_10.0", "joint_11.0",
                "joint_12.0", "joint_13.0", "joint_14.0", "joint_15.0"
        };


AllegroNode::AllegroNode(bool sim /* = false */) {
  mutex = new boost::mutex();

  // Create arrays 16 long for each of the four joint state components
  current_joint_state.position.resize(DOF_JOINTS);
  current_joint_state.velocity.resize(DOF_JOINTS);
  current_joint_state.effort.resize(DOF_JOINTS);
  current_joint_state.name.resize(DOF_JOINTS);

  // Initialize values: joint names should match URDF, desired torque and
  // velocity are both zero.
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.name[i] = jointNames[i];
    desired_torque[i] = 0.0;
    current_velocity[i] = 0.0;
    current_position_filtered[i] = 0.0;
    current_velocity_filtered[i] = 0.0;
  }

  // Get Allegro Hand information from parameter server
  // This information is found in the Hand-specific "zero.yaml" file from the allegro_hand_description package
  std::string robot_name, manufacturer, origin, serial;
  double version;
  ros::param::get("~hand_info/robot_name", robot_name);
  ros::param::get("~hand_info/which_hand", whichHand);
  ros::param::get("~hand_info/manufacturer", manufacturer);
  ros::param::get("~hand_info/origin", origin);
  ros::param::get("~hand_info/serial", serial);
  ros::param::get("~hand_info/version", version);


  // Start ROS time
  tstart = ros::Time::now();

  // Advertise current joint state publisher and subscribe to desired joint
  // states.
  joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
  joint_cmd_sub = nh.subscribe(DESIRED_STATE_TOPIC, 1, // queue size
                                &AllegroNode::desiredStateCallback, this);


  mujoco_joint_set_pub_ = nh.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/joint_set", 1);
  mujoco_sim_command_pub_ = nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim", 100);
  mujoco_sim_command_sub_ = nh.subscribe("/mujoco_ros_interface/sim_command_sim2con", 100, &AllegroNode::simCommandCallback, this);

  mujoco_joint_state_sub_ = nh.subscribe("/mujoco_ros_interface/joint_states", 1, &AllegroNode::jointStateCallback, this, ros::TransportHints().tcpNoDelay(true));
  mujoco_sim_time_sub_ = nh.subscribe("/mujoco_ros_interface/sim_time", 1, &AllegroNode::simTimeCallback, this, ros::TransportHints().tcpNoDelay(true));

  mujoco_joint_set_msg_.position.resize(DOF_JOINTS);
  mujoco_joint_set_msg_.torque.resize(DOF_JOINTS);

  mujoco_sim_time = 0.0;
  ROS_INFO("Waiting for connection with Mujoco Ros interface ");
  simready();
  ROS_INFO("Mujoco Ros interface Connected");
}

AllegroNode::~AllegroNode() {
  delete mutex;
  nh.shutdown();
}

void AllegroNode::simready()
{
    ros::Rate poll_rate(100);
    while (!mujoco_ready && ros::ok())
    {
        ros::spinOnce();
        poll_rate.sleep();
    }
    mujoco_ready = false;

}

void AllegroNode::simTimeCallback(const std_msgs::Float32ConstPtr &msg)
{
    mujoco_sim_time = msg->data;
    control_time_ = mujoco_sim_time;
}

void AllegroNode::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    for (int i = 0; i < DOF_JOINTS; i++)
    {
        current_position[i] = msg->position[i];
    }
}

void AllegroNode::simCommandCallback(const std_msgs::StringConstPtr &msg)
{

    std::string buf;
    buf = msg->data;

    ROS_INFO("CB from simulator : %s", buf.c_str());
    if (buf == "RESET")
    {
        //parameterInitialize();
        mujoco_sim_last_time = 0.0;

        mujoco_ready = true;

        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_sim_command_pub_.publish(rst_msg_);

        ros::Rate poll_rate(100);
        while (!mujoco_init_receive && ros::ok())
        {
            ros::spinOnce();
            poll_rate.sleep();
        }
        mujoco_init_receive = false;
    }

    if (buf == "INIT")
    {
        mujoco_init_receive = true;
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_sim_command_pub_.publish(rst_msg_);
        mujoco_sim_time = 0.0;
        control_time_ = 0.0;
        mujoco_reset = true;
    }
}

void AllegroNode::writeDevice()
{

    mujoco_joint_set_msg_.MODE = 1;

    for (int i = 0; i < DOF_JOINTS; i++)
    {
        mujoco_joint_set_msg_.torque[i] = desired_torque[i];
    }

    mujoco_joint_set_msg_.header.stamp = ros::Time::now();
    mujoco_joint_set_msg_.time = control_time_;
    mujoco_joint_set_pub_.publish(mujoco_joint_set_msg_);
    mujoco_sim_last_time = mujoco_sim_time;
}

void AllegroNode::wait()
{
    bool test_b = false;

    ros::Rate poll_rate(20000);
    int n = 0;

    ROS_INFO_COND(test_b, " wait loop enter");
    while ((mujoco_sim_time < (mujoco_sim_last_time + 1.0 / dyn_hz)) && ros::ok())
    { 
        ros::spinOnce();
        poll_rate.sleep();
        n++;
        if (control_time_ == 0.0)
        {
            if (mujoco_reset)
            {
                mujoco_reset = false;
                break;
            }
        }
    }
    ROS_INFO_COND(test_b, " wait loop exit with n = %d", n);
}



void AllegroNode::desiredStateCallback(const sensor_msgs::JointState &msg) {
  mutex->lock();
  desired_joint_state = msg;
  mutex->unlock();
}

void AllegroNode::publishData() {
  // current position, velocity and effort (torque) published
  current_joint_state.header.stamp = tnow;
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.position[i] = current_position_filtered[i];
    current_joint_state.velocity[i] = current_velocity_filtered[i];
    current_joint_state.effort[i] = desired_torque[i];
  }
  joint_state_pub.publish(current_joint_state);
}

void AllegroNode::updateController() {

  // Calculate loop time;
  tnow = ros::Time::now();
  dt = 1e-9 * (tnow - tstart).nsec;

  // When running gazebo, sometimes the loop gets called *too* often and dt will
  // be zero. Ensure nothing bad (like divide-by-zero) happens because of this.
  if(dt <= 0) {
    ROS_DEBUG_STREAM_THROTTLE(1, "AllegroNode::updateController dt is zero.");
    return;
  }

  tstart = tnow;

  // back-up previous joint positions:
  for (int i = 0; i < DOF_JOINTS; i++) {
    previous_position[i] = current_position[i];
    previous_position_filtered[i] = current_position_filtered[i];
    previous_position_filtered_final[i] = current_position_filtered_final[i];
    previous_velocity[i] = current_velocity[i];
  }

  // update joint positions by subscriber to mujoco
  // low-pass filtering:
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_position_filtered[i] = (0.6 * current_position_filtered[i]) +
                                    (0.198 * previous_position[i]) +
                                    (0.198 * current_position[i]);
    current_position_filtered_final[i] = (0.6 * current_position_filtered_final[i]) +
                                    (0.198 * previous_position_filtered[i]) +
                                    (0.198 * current_position_filtered[i]);
    current_velocity[i] =
            (current_position_filtered_final[i] - previous_position_filtered_final[i]) / dt;
    current_velocity_filtered[i] = (0.6 * current_velocity_filtered[i]) +
                                    (0.198 * previous_velocity[i]) +
                                    (0.198 * current_velocity[i]);
  }
  // calculate control torque:
  computeDesiredTorque();
  // set & write torque to each joint:
  // PUBLISH TO MUJOCO ///////////////////////////
  writeDevice();
  // publish joint positions to ROS topic:
  publishData();
  frame++;

  if (lEmergencyStop < 0) {
    // Stop program when Allegro Hand is switched off
    ROS_ERROR("Allegro Hand Node is Shutting Down! (Emergency Stop)");
    ros::shutdown();
  }
}

// Interrupt-based control is not recommended by SimLab. I have not tested it.
void AllegroNode::timerCallback(const ros::TimerEvent &event) {
  updateController();
}

ros::Timer AllegroNode::startTimerCallback() {
  ros::Timer timer = nh.createTimer(ros::Duration(0.001),
                                    &AllegroNode::timerCallback, this);
  return timer;
}
