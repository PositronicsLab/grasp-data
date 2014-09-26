#ifndef _GAZEBO_UR10_SCHUNK_CONTROLLER_H_
#define _GAZEBO_UR10_SCHUNK_CONTROLLER_H_

//-----------------------------------------------------------------------------

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include "log.h"

//-----------------------------------------------------------------------------

class ur10_schunk_controller_c : public gazebo::ModelPlugin {

private:
  // the reference so that this ship is inserted into gazebo's callback system
  gazebo::event::ConnectionPtr _updateConnection;
  // the gazebo reference to the world in which the ship is located
  gazebo::physics::WorldPtr _world;

  gazebo::physics::ModelPtr _model;

  gazebo::physics::ModelPtr _target;
  bool _target_attached;

  // UR10 arm
  gazebo::physics::LinkPtr _base; 
  gazebo::physics::LinkPtr _shoulder;
  gazebo::physics::LinkPtr _upperarm;
  gazebo::physics::LinkPtr _forearm;
  gazebo::physics::LinkPtr _wrist1;
  gazebo::physics::LinkPtr _wrist2;
  gazebo::physics::LinkPtr _wrist3;
  
  gazebo::physics::JointPtr _shoulder_pan_actuator;   // shoulder to base
  gazebo::physics::JointPtr _shoulder_lift_actuator;  // upperarm to shoulder
  gazebo::physics::JointPtr _elbow_actuator;          // forearm to upperarm
  gazebo::physics::JointPtr _wrist1_actuator;         // wrist1 to forearm
  gazebo::physics::JointPtr _wrist2_actuator;         // wrist2 to wrist1
  gazebo::physics::JointPtr _wrist3_actuator;         // wrist3 to wrist2

  // Schunk hand
  gazebo::physics::LinkPtr _hand; 
  gazebo::physics::LinkPtr _finger_l;
  gazebo::physics::LinkPtr _finger_r;

  gazebo::physics::JointPtr _finger_actuator_l;
  gazebo::physics::JointPtr _finger_actuator_r;

  boost::shared_ptr<log_c> slip_log;

  double compute_slip( void );

  double _base_slip;

public:
  ur10_schunk_controller_c( void );
  virtual ~ur10_schunk_controller_c( void );

  // Gazebo callback.  Called when the simulation is starting up
  virtual void Load( gazebo::physics::ModelPtr model, sdf::ElementPtr sdf );

  // Gazebo callback.  Called whenever the simulation advances a timestep
  virtual void Update( );

  // Gazebo callback.  Called whenever the simulation is reset
  //virtual void Reset( );

};

//-----------------------------------------------------------------------------

#endif // _GAZEBO_UR10_SCHUNK_CONTROLLER_H_
