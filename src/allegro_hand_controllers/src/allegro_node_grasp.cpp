#include "allegro_node_grasp.h"

#include "bhand/BHand.h"

// The only topic specific to the 'grasp' controller is the envelop torque.
const std::string ENVELOP_TORQUE_TOPIC = "allegroHand/envelop_torque";

double sum = 0.0;
double kd_arr[5000] = {0.0};

int kd_count = 0;

const int reg_size = 100;
double y[reg_size] = {0.0};
double pterm[reg_size] = {0.0};
double dterm[reg_size] = {0.0};
// double kp[DOF_JOINTS] = {0.0};
// double kd[DOF_JOINTS] = {0.0};
int reg_count = 0;
int kindex = 1;

double iterm[16] = {0.0};
double k_i[16] = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0};

// Define a map from string (received message) to eMotionType (Bhand controller grasp).
std::map<std::string, eMotionType> bhand_grasps = {
        {"home",     eMotionType_HOME},
        {"ready",    eMotionType_READY},  // ready position
        {"grasp_3",  eMotionType_GRASP_3},  // grasp with 3 fingers
        {"grasp_4",  eMotionType_GRASP_4},  // grasp with 4 fingers
        {"pinch_it", eMotionType_PINCH_IT},  // pinch, index & thumb
        {"pinch_mt", eMotionType_PINCH_MT},  // pinch, middle & thumb
        {"envelop",  eMotionType_ENVELOP},  // envelop grasp (power-y)
        {"off",      eMotionType_NONE},  // turn joints off
        {"gravcomp", eMotionType_GRAVITY_COMP},  // gravity compensation
        // These ones do not appear to do anything useful (or anything at all):
        // {"pregrasp", eMotionType_PRE_SHAPE},  // not sure what this is supposed to do.
        // {"move_object", eMotionType_OBJECT_MOVING},
        // {"move_fingertip", eMotionType_FINGERTIP_MOVING}
};

AllegroNodeGrasp::AllegroNodeGrasp()
        : AllegroNode() {

  initController(whichHand);

  joint_cmd_sub = nh.subscribe(
          DESIRED_STATE_TOPIC, 3, &AllegroNodeGrasp::setJointCallback, this);
  lib_cmd_sub = nh.subscribe(
          LIB_CMD_TOPIC, 1, &AllegroNodeGrasp::libCmdCallback, this);

  envelop_torque_sub = nh.subscribe(
          ENVELOP_TORQUE_TOPIC, 1, &AllegroNodeGrasp::envelopTorqueCallback,
          this);
}

AllegroNodeGrasp::~AllegroNodeGrasp() {
  delete pBHand;
}

void AllegroNodeGrasp::libCmdCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());
  const std::string lib_cmd = msg->data;

  // Main behavior: apply the grasp directly from the map. Secondary behaviors can still be handled
  // normally (case-by-case basis), note these should *not* be in the map.
  auto itr = bhand_grasps.find(msg->data);
  if (itr != bhand_grasps.end()) {
    pBHand->SetMotionType(itr->second);
    ROS_INFO("motion type = %d", itr->second);
  } else if (lib_cmd.compare("pdControl") == 0) {
    // Desired position only necessary if in PD Control mode
    pBHand->SetJointDesiredPosition(desired_position);
    pBHand->SetMotionType(eMotionType_JOINT_PD);
  } else if (lib_cmd.compare("save") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = current_position[i];
  } else {
    ROS_WARN("Unknown commanded grasp: %s.", lib_cmd.c_str());
  }
}

// Called when a desired joint position message is received
void AllegroNodeGrasp::setJointCallback(const sensor_msgs::JointState &msg) {
  mutex->lock();

  for (int i = 0; i < DOF_JOINTS; i++)
    desired_position[i] = msg.position[i];
  mutex->unlock();

  pBHand->SetJointDesiredPosition(desired_position);
  pBHand->SetMotionType(eMotionType_JOINT_PD);
}

// The grasp controller can set the desired envelop grasp torque by listening to
// Float32 messages on ENVELOP_TORQUE_TOPIC ("allegroHand/envelop_torque").
void AllegroNodeGrasp::envelopTorqueCallback(const std_msgs::Float32 &msg) {
  const double torque = msg.data;
  ROS_INFO("Setting envelop torque to %.3f.", torque);
  pBHand->SetEnvelopTorqueScalar(torque);
}

void AllegroNodeGrasp::computeDesiredTorque() {

  mutex->lock();
  // compute control torque using Bhand library

  pBHand->SetMotionType(eMotionType_JOINT_PD);

  // pBHand->SetMotionType(eMotionType_GRAVITY_COMP);

  pBHand->SetJointPosition(current_position_filtered);

  // BHand lib control updated with time stamp
  pBHand->UpdateControl((double) frame * ALLEGRO_CONTROL_TIME_INTERVAL);

  // Necessary torque obtained from Bhand lib
  pBHand->GetJointTorque(desired_torque);

  // std::cout << pBHand->_mass[0][0] << std::endl;
  // for(int i=0;i<4;i++){
  //   for(int j=0;j<4;j++){
  //     iterm[i*4+j] += pBHand->_q_filtered[i][j] - pBHand->_q_des[i][j];
  //     if(iterm[i*4+j] > 100.0) iterm[i*4+j] = 100.0;
  //     else if(iterm[i*4+j] < -100.0) iterm[i*4+j] = -100.0;
  //     desired_torque[i*4+j] -= iterm[i*4+j]*k_i[i*4+j]/30000.0;
  //   }
  // }
  // std::cout << pBHand->_q_filtered[0][1] - pBHand->_q_des[0][1] << std::endl;
  // std::cout << iterm[1] << std::endl;

  // std::cout << "joint0 " << pBHand->_J[0][0][0] << " " << pBHand->_J[0][1][0] << " " << pBHand->_J[0][2][0] << std::endl;
  // std::cout << "joint1 " << pBHand->_J[0][0][1] << " " << pBHand->_J[0][1][1] << " " << pBHand->_J[0][2][1] << std::endl;
  // std::cout << "joint2 " << pBHand->_J[0][0][2] << " " << pBHand->_J[0][1][2] << " " << pBHand->_J[0][2][2] << std::endl;
  // std::cout << "joint3 " << pBHand->_J[0][0][3] << " " << pBHand->_J[0][1][3] << " " << pBHand->_J[0][2][3] << std::endl;

  // // std::cout << "m1 " << pBHand->_mass[0][1] << "m2 " << pBHand->_mass[0][2] << "m3 " << pBHand->_mass[0][3] << std::endl;

  double num1 = pBHand->_mass[0][0] * pBHand->_J[0][0][0] + pBHand->_mass[0][1] * pBHand->_J[0][0][1] + pBHand->_mass[0][2] * pBHand->_J[0][0][2] \
                + pBHand->_mass[0][3] * pBHand->_J[0][0][3];
  double num2 = pBHand->_mass[0][0] * pBHand->_J[0][1][0] + pBHand->_mass[0][1] * pBHand->_J[0][1][1] + pBHand->_mass[0][2] * pBHand->_J[0][1][2] \
                + pBHand->_mass[0][3] * pBHand->_J[0][1][3];
  double num3 = pBHand->_mass[0][0] * pBHand->_J[0][2][0] + pBHand->_mass[0][1] * pBHand->_J[0][2][1] + pBHand->_mass[0][2] * pBHand->_J[0][2][2] \
                + pBHand->_mass[0][3] * pBHand->_J[0][2][3];
  // double num2 = pBHand->_mass[0][2] * pBHand->_J[0][2][2] \
  //               + pBHand->_mass[0][3] * pBHand->_J[0][2][3];
  // double num3 = pBHand->_mass[0][1] * pBHand->_J[0][0][1] + pBHand->_mass[0][2] * pBHand->_J[0][0][2] \
  //               + pBHand->_mass[0][3] * pBHand->_J[0][0][3];
  // double num4 = pBHand->_mass[0][2] * pBHand->_J[0][0][2] \
  //               + pBHand->_mass[0][3] * pBHand->_J[0][0][3];
  // double numG = pBHand->_G[0][1];
  double desT = desired_torque[0];
  // std::cout << (desT/num1) << " " << (desT/num2) << " " << (desT/num3) << std::endl;
  // std::cout << numG << std::endl;
  // std::cout << num1/numG << " " << num2/numG << " " << num3/numG << " " << num4/numG << std::endl;
  // std::cout << num1/desT << " " << num2/desT << " " << num3/desT << " " << num4/desT << std::endl;
  // double x[4], y[4], z[4];

  // pBHand->GetFKResult(x,y,z);

  // for(int i = 0; i < 4; i++){
  //   for(int j = 0; j < 4; j++){
  //     std::cout << i << " " << j << " " << pBHand->_kd[i][j] << std::endl;
  //   }
  // }
  
  
  // std::cout << "x: " << x[0] << " y: " << y[0] << " z: " << z[0] << std::endl;
  
  // std::cout << pBHand->_q_filtered[0][1] << " " << pBHand->_qdot[0][1] << std::endl;

  // std::cout << pBHand->_q_filtered[0][1] << " " << pBHand->_qdot[0][1] << std::endl;

  // pterm[reg_count] = pBHand->_q_des[0][1] - pBHand->_q_filtered[0][1];
  // dterm[reg_count] = pBHand->_qdot_filtered[0][1];
  // y[reg_count] = desired_torque[kindex] * canDevice->torqueConversion();

  // reg_count++;
  // if(reg_count == reg_size)
  // {
  //   reg_count = 0;
  //   double a11 = 0.0; double a12 = 0.0; double a21 = 0.0; double a22 = 0.0;
  //   double c1 = 0.0; double c2 = 0.0;

  //   for(int j = 0; j < reg_size; j++){
  //     a11 += pterm[j]*pterm[j];
  //     a12 += pterm[j]*dterm[j];
  //     a21 += dterm[j]*pterm[j];
  //     a22 += dterm[j]*dterm[j];
  //     c1 += pterm[j]*y[j];
  //     c2 += dterm[j]*y[j];
  //   }
  //   double det = a11*a22 - a12*a21;
  //   double b11  = 0.0; double b12 = 0.0; double b21 = 0.0; double b22 = 0.0;
  //   b11 = a22 / det; b12 = -a12/det; b21 = -a21/det; b22 = a11/det;
  //   double kp = b11*c1+b12*c2;
  //   double kd = b21*c1+b22*c2;
  //   // std::cout << std::fixed << std::setprecision(2) << "kp: " << kp << " kd: " << std::setprecision(6) << kd << std::endl;
  //   reg_count = 0;
  // }

  // std::cout << pBHand->_qdot_pre[0][1] << " " << pBHand->_qdot[0][1] << " " << pBHand->_qdot_filtered[0][1] << std::endl;
  // std::cout << "R" << std::endl;
  
  // std::cout << pBHand->_R[0] << " " << pBHand->_R[1] << " " << pBHand->_R[2] << std::endl;
  // std::cout << pBHand->_R[3] << " " << pBHand->_R[4] << " " << pBHand->_R[5] << std::endl;
  // std::cout << pBHand->_R[6] << " " << pBHand->_R[7] << " " << pBHand->_R[8] << std::endl;
 


  //ROS_INFO("desired torque = %.3f %.3f %.3f %.3f", desired_torque[0], desired_torque[1], desired_torque[2], desired_torque[3]);


  /*
  double kp[DOF_JOINTS] = {740.0, 1200.0, 1350.0, 740.0, 740.0, 1200.0, 1350.0, 740.0, \
                           740.0, 1200.0, 1350.0, 740.0, 1470.0, 1040.0, 890.0, 890.0};
  
  if (true) {
    // Control joint positions: compute the desired torques (PD control).
    double error[DOF_JOINTS] = {0.0};
    
    double kd[DOF_JOINTS] = {0.0};
    for (int i = 0; i < DOF_JOINTS; i++) {
      error[i] = desired_position[i] - current_position_filtered[i];
      kd[i] = -(desired_torque[i] * canDevice->torqueConversion() - kp[i] * error[i]) / current_velocity_filtered[i];
      // std::cout << i << ": " << kp[i] << " ";
    }

    int kindex = 1;
    sum -= kd_arr[kd_count];
    kd_arr[kd_count] = kd[kindex];
    sum += kd_arr[kd_count];
    kd_count++;
    if(kd_count == 5000) kd_count = 0;

    // std::cout << std::endl;

    std::cout << kindex << " " << kd_count << ": " << sum/5000.0 << std::endl;
  }
  */
  mutex->unlock();
}

void AllegroNodeGrasp::initController(const std::string &whichHand) {
  // Initialize BHand controller
  if (whichHand.compare("left") == 0) {
    pBHand = new BHand(eHandType_Left);
    ROS_WARN("CTRL: Left Allegro Hand controller initialized.");
  }
  else {
    pBHand = new BHand(eHandType_Right);

    // double R[9] = {0,1,0,0,0,0,0,0,1};
    // pBHand->SetOrientation(R);
    // pBHand->SetOrientation(1.57,0.0,0.0);
    // std::cout << pBHand->_R[0] << " " << pBHand->_R[1] << " " << pBHand->_R[2] << std::endl;
    // std::cout << pBHand->_R[3] << " " << pBHand->_R[4] << " " << pBHand->_R[5] << std::endl;
    // std::cout << pBHand->_R[6] << " " << pBHand->_R[7] << " " << pBHand->_R[8] << std::endl;

    ROS_WARN("CTRL: Right Allegro Hand controller initialized.");
  }
  pBHand->SetTimeInterval(ALLEGRO_CONTROL_TIME_INTERVAL);
  pBHand->SetMotionType(eMotionType_NONE);

  // sets initial desired pos at start pos for PD control
  for (int i = 0; i < DOF_JOINTS; i++)
    desired_position[i] = current_position[i];

  printf("*************************************\n");
  printf("         Grasp (BHand) Method        \n");
  printf("-------------------------------------\n");
  printf("         Every command works.        \n");
  printf("*************************************\n");
}

void AllegroNodeGrasp::doIt(bool polling) {
  if (polling) {
    ROS_INFO("Polling = true.");
    while (ros::ok()) {
      updateController();
      ros::spinOnce();
    }
  } else {
    ROS_INFO("Polling = false.");

    // Timer callback (not recommended).
    ros::Timer timer = startTimerCallback();
    ros::spin();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "allegro_hand_core_grasp");
  AllegroNodeGrasp grasping;

  bool polling = false;
  if (argv[1] == std::string("true")) {
    polling = true;
  }
  ROS_INFO("Start controller with polling = %d", polling);

  grasping.doIt(polling);
}
