#ifndef __ALLEGRO_NODE_SIM_H__
#define __ALLEGRO_NODE_SIM_H__

#include "allegro_node_mj.h"


// Joint-space PD control of the Allegro hand.
//
// Allows you to save a position and command it to the hand controller.
// Controller gains are loaded from the ROS parameter server.
class AllegroNodeSim : public AllegroNode {

 public:
  AllegroNodeSim();

  ~AllegroNodeSim();

  // Main spin code: just waits for messages.
  void doIt();

  // Loads all gains and initial positions from the parameter server.
  void initController(const std::string &whichHand);

  // PD control happens here.
  void computeDesiredTorque();

 protected:

  // If this flag is true, the hand will be controlled (either in joint position
  // or joint torques). If false, desired torques will all be zero.
  bool control_hand_ = false;
 
};

#endif  // __ALLEGRO_NODE_SIM_H__
