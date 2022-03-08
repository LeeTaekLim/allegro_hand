//
// Created by felixd on 10/1/15.
//

#ifndef PROJECT_ALLEGRO_NODE_COMMON_H
#define PROJECT_ALLEGRO_NODE_COMMON_H

//#include "math_type_define.h"

#include <string>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

#include <std_msgs/Float32.h>
#include <mujoco_ros_msgs/SensorState.h>
#include <mujoco_ros_msgs/JointSet.h>


#define ALLEGRO_CONTROL_TIME_INTERVAL 0.003
#define DOF_JOINTS 16

// Topic names: current & desired JointState, named grasp to command.
const std::string JOINT_STATE_TOPIC = "allegroHand/joint_states";
const std::string DESIRED_STATE_TOPIC = "allegroHand/joint_cmd";

class AllegroNode {
 public:

  AllegroNode(bool sim = true);

  virtual ~AllegroNode();

  void publishData();

  void desiredStateCallback(const sensor_msgs::JointState &desired);

  virtual void updateController();

  // This is the main method that must be implemented by the various
  // controller nodes.
  virtual void computeDesiredTorque() {
    ROS_ERROR("Called virtual function!");
  };

  ros::Timer startTimerCallback();

  void timerCallback(const ros::TimerEvent &event);

 protected:

  double current_position[DOF_JOINTS] = {0.0};
  double previous_position[DOF_JOINTS] = {0.0};

  double current_position_filtered[DOF_JOINTS] = {0.0};
  double previous_position_filtered[DOF_JOINTS] = {0.0};

  double current_position_filtered_final[DOF_JOINTS] = {0.0};
  double previous_position_filtered_final[DOF_JOINTS] = {0.0};
  
  double current_velocity[DOF_JOINTS] = {0.0};
  double previous_velocity[DOF_JOINTS] = {0.0};
  double current_velocity_filtered[DOF_JOINTS] = {0.0};

  double desired_torque[DOF_JOINTS] = {0.0};

  std::string whichHand;  // Right or left hand.

  // ROS stuff
  ros::NodeHandle nh;
  ros::Publisher joint_state_pub;
  ros::Subscriber joint_cmd_sub;

  // Store the current and desired joint states.
  sensor_msgs::JointState current_joint_state;
  sensor_msgs::JointState desired_joint_state;

  // ROS Time
  ros::Time tstart;
  ros::Time tnow;
  double dt;

  boost::mutex *mutex;

  // Flags
  int lEmergencyStop = 0;
  long frame = 0;

// Mujoco
 public:
  virtual void writeDevice();
  virtual void wait();
 private: // CALLBACK
  void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);
  void simCommandCallback(const std_msgs::StringConstPtr &msg);
  void simTimeCallback(const std_msgs::Float32ConstPtr &msg);
  void simready();

 private:
  ros::Publisher mujoco_joint_set_pub_;
  ros::Publisher mujoco_sim_command_pub_;

  ros::Subscriber mujoco_joint_state_sub_;
  ros::Subscriber mujoco_sensor_state_sub_;
  ros::Subscriber mujoco_sim_command_sub_;
  ros::Subscriber mujoco_sim_time_sub_;

  //sensor_msgs::JointState mujoco_joint_set_msg_;
  mujoco_ros_msgs::JointSet mujoco_joint_set_msg_;

 public:
  bool sim_runnung;
  bool mujoco_ready = false;
  bool mujoco_init_receive = false;

  bool mujoco_reset = false;

  float mujoco_sim_time;
  float mujoco_sim_last_time;
  
  int dyn_hz;
  double control_time_;

};

#endif //PROJECT_ALLEGRO_NODE_COMMON_H
