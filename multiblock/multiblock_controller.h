#ifndef _GAZEBO_MULTIBLOCK_CONTROLLER_H_
#define _GAZEBO_MULTIBLOCK_CONTROLLER_H_

//-----------------------------------------------------------------------------

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include "log.h"

//-----------------------------------------------------------------------------

#define NEWTONS_PER_TARGET 200.0

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

  // left gripper energy constants
  std::vector<gazebo::math::Vector3> _targets_c_v_l;
  std::vector<gazebo::math::Vector3> _targets_c_omega_l;
  // right gripper energy constants
  std::vector<gazebo::math::Vector3> _targets_c_v_r;
  std::vector<gazebo::math::Vector3> _targets_c_omega_r;

  // the force to be applied at each of the grippers
  double _force_at_gripper;

  gazebo::math::Vector3 _gripper_l_initial;
  gazebo::math::Vector3 _gripper_r_initial;

  double gripper_travel( gazebo::math::Vector3 initial_pos, gazebo::math::Vector3 current_pos );
  //void assess_closure( double sim_time );

  boost::shared_ptr<log_c> energy_log;

  static double energy( gazebo::physics::LinkPtr gripper, gazebo::physics::ModelPtr grip_target, gazebo::math::Vector3 c_v, gazebo::math::Vector3 c_omega, double target_mass, gazebo::math::Matrix3 target_I_tensor, double dt );

  static double energy( double mass, gazebo::math::Matrix3 I_tensor, double dt, gazebo::math::Pose current_pose, gazebo::math::Pose desired_pose, gazebo::math::Vector3 current_linvel, gazebo::math::Vector3 desired_linvel, gazebo::math::Vector3 current_angvel, gazebo::math::Vector3 desired_angvel );

  static gazebo::math::Vector3 to_omega( gazebo::math::Quaternion q, gazebo::math::Quaternion q_dot );
  static gazebo::math::Quaternion deriv(gazebo::math::Quaternion q, gazebo::math::Vector3 w);

  double previous_t;

public:
  robot_c( void );
  virtual ~robot_c( void );

  void Load( gazebo::physics::ModelPtr model, sdf::ElementPtr sdf );
  
  void Update( void );
};

//-----------------------------------------------------------------------------
#endif // _GAZEBO_MULTIBLOCK_CONTROLLER_H_
