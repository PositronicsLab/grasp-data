#ifndef _GAZEBO_MULTIBLOCK_CONTROLLER_H_
#define _GAZEBO_MULTIBLOCK_CONTROLLER_H_

//-----------------------------------------------------------------------------

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

//-----------------------------------------------------------------------------

#define NEWTONS_PER_TARGET 100.0

//-----------------------------------------------------------------------------

class robot_c : public gazebo::ModelPlugin {
private:
  gazebo::event::ConnectionPtr _update_connection;

  // reference to the robot model
  gazebo::physics::ModelPtr _model;
  // reference to the world
  gazebo::physics::WorldPtr _world;

  // reference to the left gripper link
  gazebo::physics::LinkPtr _gripper_l;
  // reference to the right gripper link
  gazebo::physics::LinkPtr _gripper_r;

  // reference to the left gripper actuator joint
  gazebo::physics::JointPtr _actuator_l;
  // reference to the right gripper actuator joint
  gazebo::physics::JointPtr _actuator_r;

  // the list of block targets in the grasp
  gazebo::physics::Model_V _targets;

  // the force to be applied at each of the grippers
  double _force_at_gripper;

public:
  robot_c( void );
  virtual ~robot_c( void );

  void Load( gazebo::physics::ModelPtr model, sdf::ElementPtr sdf );
  
  void Update( void );
};

//-----------------------------------------------------------------------------
#endif // _GAZEBO_MULTIBLOCK_CONTROLLER_H_
