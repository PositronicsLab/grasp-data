#include "multiblock_controller.h"

//-----------------------------------------------------------------------------

GZ_REGISTER_MODEL_PLUGIN( robot_c )

//-----------------------------------------------------------------------------
robot_c::robot_c( void ) { 

}

//-----------------------------------------------------------------------------
robot_c::~robot_c( void ) {
  gazebo::event::Events::DisconnectWorldUpdateBegin( _update_connection );
}

//-----------------------------------------------------------------------------
void robot_c::Load( gazebo::physics::ModelPtr model, sdf::ElementPtr sdf ) {

  _model = model;
  _world = model->GetWorld();

  // locate the robot links
  _gripper_l = model->GetLink("gripper_l");
  if( !_gripper_l ) {
    gzerr << "Unable to find link: gripper_l\nPlugin failed to load\n";
    return;
  }

  _gripper_r = model->GetLink("gripper_r");
  if( !_gripper_r ) {
    gzerr << "Unable to find link: gripper_r\nPlugin failed to load\n";
    return;
  }

  // locate the robot actuators
  _actuator_l = model->GetJoint("actuator_l");
  if( !_actuator_l ) {
    gzerr << "Unable to find joint: actuator_l\nPlugin failed to load\n";
    return;
  } 

  _actuator_r = model->GetJoint("actuator_r");
  if( !_actuator_r ) {
    gzerr << "Unable to find joint: actuator_r\nPlugin failed to load\n";
    return;
  } 

  // Get targets
  gazebo::physics::Model_V targets = _world->GetModels();
  for( unsigned i = 0; i < targets.size(); i++ ) 
    if( targets[i]->GetName() == "block" ) _targets.push_back( targets[i] );
  printf( "found %u block targets\n", _targets.size() );

  // Compute the force at the gripper
  double target_count = (double) _targets.size();
  _force_at_gripper = target_count * NEWTONS_PER_TARGET / 2.0; 
  printf( "force to be applied at each gripper: %f N\n", _force_at_gripper );

  // register the update callback 
  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind( &robot_c::Update, this ) );

  // FIN
  printf( "multiblock_controller has initialized\n" );
}

//-----------------------------------------------------------------------------
void robot_c::Update( void ) {
  // apply the force to the gripper actuators
  _actuator_l->SetForce( 0, _force_at_gripper );
  _actuator_r->SetForce( 0,-_force_at_gripper );
}

//-----------------------------------------------------------------------------
