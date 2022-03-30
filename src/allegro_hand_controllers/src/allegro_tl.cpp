#include "allegro_tl.h"


#define INDEX_FINGER 0
#define MIDDLE_FINGER 4
#define RING_FINGER 8

AllegroTL::AllegroTL(){
    start_t = ros::Time::now();
}

void AllegroTL::setDesiredJointState(sensor_msgs::JointState& desired_joint_state, ros::Time& start_t){
    t_ = tnow-start_t;
    t = t_.nsec*1e-9+t_.sec;
    desired_joint_state.position[MIDDLE_FINGER] = 0;desired_joint_state.position[MIDDLE_FINGER+2] = 0;
    desired_joint_state.position[INDEX_FINGER] = 0;desired_joint_state.position[INDEX_FINGER+2] = 0;
    desired_joint_state.position[MIDDLE_FINGER+3] = 0;
    desired_joint_state.position[INDEX_FINGER+3] = 0;
    desired_joint_state.position[MIDDLE_FINGER+1] = DEGREES_TO_RADIANS(8)*sin(2*M_PI*1*t);
    desired_joint_state.position[INDEX_FINGER+1] = -DEGREES_TO_RADIANS(8)*sin(2*M_PI*1*t);

}


void AllegroTL::setDesiredTorque(double[] desired_torque){
    double error;
    for (int i = 0; i < DOF_JOINTS; i++) {
    error = desired_joint_state.position[i] - current_position_filtered[i];
    desired_torque[i] = 1.0/canDevice->torqueConversion() *
            (k_p[i] * error - k_d[i] * current_velocity_filtered[i]);
    }
}