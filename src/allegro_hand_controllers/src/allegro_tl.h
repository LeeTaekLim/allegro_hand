#ifndef ALLEGRO_TL
#define ALLEGRO_TL

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "allegro_node_pd.h"
#include <cmath>

class AllegroTL : public AllegroNodePD{
public:
    AllegroTL();

    void setDesiredJointState(sensor_msgs::JointState& desired_joint_state);
    void setDesiredTorque(double[] desired_torque);

    ros::Time start_t;
    double t;


};




#endif