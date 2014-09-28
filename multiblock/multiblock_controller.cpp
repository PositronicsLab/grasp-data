#include "multiblock_controller.h"

#include <sstream>

#define MAX_SIM_TIME 100.0

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
/*
  gazebo::math::Quaternion a( 1.5, 0, 0, 0);
  printf( "%f,%f,%f,%f\n", a.x, a.y, a.z, a.w );
  gazebo::math::Quaternion b( 1.5, 0, 0, 0);
  printf( "%f,%f,%f,%f\n", b.x, b.y, b.z, b.w );
  gazebo::math::Quaternion c = a + b;
  printf( "%f,%f,%f,%f\n", c.x, c.y, c.z, c.w );
*/

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

  // get the initial positions of the grippers for failure detection
  _gripper_l_initial = _gripper_l->GetWorldPose().pos;
  _gripper_r_initial = _gripper_r->GetWorldPose().pos;

  // Get targets
  gazebo::physics::Model_V targets = _world->GetModels();
  for( unsigned i = 0; i < targets.size(); i++ ) { 
    if( targets[i]->GetName() == "block" ) {
      gazebo::physics::ModelPtr target = targets[i];
      _targets.push_back( target );

      gazebo::math::Vector3 c_v, c_omega;
      // left gripper energy constants
      c_v = _gripper_l->GetWorldPose().pos - target->GetWorldPose().pos;
      _targets_c_v_l.push_back( c_v );
      c_omega = to_omega( _gripper_l->GetWorldPose().rot, target->GetWorldPose().rot );
      _targets_c_omega_l.push_back( c_omega );
      // right gripper energy constants
      c_v = _gripper_r->GetWorldPose().pos - target->GetWorldPose().pos;
      _targets_c_v_r.push_back( c_v );
      c_omega = to_omega( _gripper_r->GetWorldPose().rot, target->GetWorldPose().rot );
      _targets_c_omega_r.push_back( c_omega );
    }
  }
  printf( "found %u block targets\n", _targets.size() );

  // Compute the force at the gripper
  double target_count = (double) _targets.size();
  _force_at_gripper = target_count * NEWTONS_PER_TARGET / 2.0; 
  printf( "force to be applied at each gripper: %f N\n", _force_at_gripper );

  // Logging
  std::string engine_name = _world->GetPhysicsEngine()->GetType();
  std::string world_name = _world->GetName();
  std::stringstream ss_logname;
  ss_logname << engine_name << "_" << world_name << ".log";

  energy_log = boost::shared_ptr<log_c>( new log_c( ss_logname.str() ) );
  energy_log->open();

  // register the update callback 
  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind( &robot_c::Update, this ) );

  previous_t = 0.0;

  // FIN
  printf( "multiblock_controller has initialized\n" );
}

//-----------------------------------------------------------------------------
void robot_c::Update( void ) {
  static bool first_pass = true;

  double t = _world->GetSimTime().Double();
  double dt = t - previous_t;
  double real_time = _world->GetRealTime().Double(); 

  std::stringstream data;
  std::vector<double> avgKEs;

  //std::cout << "t : " << t << std::endl;
  data << t << "," << real_time;
  // compute the energy of each of the targets
  for( unsigned i = 0; i < _targets.size(); i++ ) {
    gazebo::physics::ModelPtr target = _targets[i];
    gazebo::physics::LinkPtr body = target->GetLink("body");
    assert( body );

    // get the inertial properties of the box
    gazebo::physics::InertialPtr gz_inertial = body->GetInertial();
    double m = gz_inertial->GetMass();
    gazebo::math::Vector3 pm = gz_inertial->GetPrincipalMoments();
    gazebo::math::Matrix3 I( pm.x, 0.0, 0.0, 0.0, pm.y, 0.0, 0.0, 0.0, pm.z);

    double KE_l = energy( _gripper_l, target, _targets_c_v_l[i], _targets_c_omega_l[i], m, I, dt );
    double KE_r = energy( _gripper_r, target, _targets_c_v_r[i], _targets_c_omega_r[i], m, I, dt );

    double avg_KE = (KE_l + KE_r) / 2.0;
    avgKEs.push_back( avg_KE );

    //std::cout << i << " KE[l,r,avg],pos : " << KE_l<< "," << KE_r << "," << avg_KE << "," << target->GetWorldPose().pos << std::endl;
    data << "," << i << "," << KE_l << "," << KE_r << "," << avg_KE << "," << target->GetWorldPose().pos;
  }
  //std::cout << "--------------------" << std::endl;
  data << std::endl;
  energy_log->write( data.str() );

  // check for exit condition
  for( std::vector<double>::iterator it = avgKEs.begin(); it != avgKEs.end(); it++ ) {
    if( fabs(*it) > 1.0e7 ) exit( 0 );
    if( t >= MAX_SIM_TIME ) exit( 0 );
  }


  // apply the force to the gripper actuators
  _actuator_l->SetForce( 0, _force_at_gripper );
  _actuator_r->SetForce( 0,-_force_at_gripper );

  previous_t = t;

  first_pass = false;
}

//-----------------------------------------------------------------------------
double robot_c::gripper_travel( gazebo::math::Vector3 initial_pos, gazebo::math::Vector3 current_pos ) {
  gazebo::math::Vector3 v_travel = current_pos - initial_pos;
  return v_travel.GetLength();
}

/*
//-----------------------------------------------------------------------------
void robot_c::assess_closure( double sim_time ) {
  gazebo::math::Vector3 gripper_l_current = _gripper_l->GetWorldPose().pos;
  gazebo::math::Vector3 gripper_r_current = _gripper_r->GetWorldPose().pos;

  double l_travel = gripper_travel( _gripper_l_initial, gripper_l_current );
  double r_travel = gripper_travel( _gripper_r_initial, gripper_r_current );

  std::stringstream ss_traveldata;
  ss_traveldata << sim_time << "," << l_travel << "," << r_travel << std::endl;
  travel_log->write( ss_traveldata.str() );

  if( (l_travel > 0.05 && r_travel > 0.05) ||  
      (l_travel > 0.1 ) || 
      (r_travel > 0.1 ) ) {
    printf( "grip has failed\n" );
    travel_log->close();
    exit( 0 );
  }
}
*/
//-----------------------------------------------------------------------------
gazebo::math::Vector3 robot_c::to_omega( gazebo::math::Quaternion q, gazebo::math::Quaternion qd ) {
  gazebo::math::Vector3 omega;
  omega.x = 2 * (-q.x * qd.w + q.w * qd.x - q.z * qd.y + q.y * qd.z);
  omega.y = 2 * (-q.y * qd.w + q.z * qd.x + q.w * qd.y - q.x * qd.z);
  omega.z = 2 * (-q.z * qd.w - q.y * qd.x + q.x * qd.y + q.w * qd.z);
  return omega;

}

//-----------------------------------------------------------------------------
gazebo::math::Quaternion robot_c::deriv(gazebo::math::Quaternion q, gazebo::math::Vector3 w) {
  gazebo::math::Quaternion qd;

  qd.w = .5 * (-q.x * w.x - q.y * w.y - q.z * w.z); 
  qd.x = .5 * (+q.w * w.x + q.z * w.y - q.y * w.z);
  qd.y = .5 * (-q.z * w.x + q.w * w.y + q.x * w.z);
  qd.z = .5 * (+q.y * w.x - q.x * w.y + q.w * w.z);

  return qd;
}

//-----------------------------------------------------------------------------
double robot_c::energy( gazebo::physics::LinkPtr gripper, gazebo::physics::ModelPtr grip_target, gazebo::math::Vector3 c_v, gazebo::math::Vector3 c_omega, double m, gazebo::math::Matrix3 I, double dt ) {

    gazebo::math::Pose x = grip_target->GetWorldPose();

    //desired pose
    gazebo::math::Pose x_des;
    gazebo::math::Vector3 delta = gripper->GetWorldPose().pos - grip_target->GetWorldPose().pos;
    c_v = x.rot.RotateVector( c_v );
    x_des.pos = x.pos + (delta - c_v);

    gazebo::math::Quaternion dq = deriv( x.rot, c_omega );
    x_des.rot = x.rot + dq;
    x_des.rot.Normalize();

    gazebo::math::Vector3 linvel = grip_target->GetWorldLinearVel();
    // desired linear velocity
    gazebo::math::Vector3 linvel_des = gripper->GetWorldLinearVel();

    gazebo::math::Vector3 angvel = grip_target->GetWorldAngularVel();   
    // desired angular velocity
    gazebo::math::Vector3 angvel_des = gripper->GetWorldAngularVel();

    //if( !first_pass ) {
    double KE = energy( m, I, dt, x, x_des, linvel, linvel_des, angvel, angvel_des );
    return KE;
}

//-----------------------------------------------------------------------------
double robot_c::energy( double m, gazebo::math::Matrix3 I, double dt, gazebo::math::Pose current_pose, gazebo::math::Pose desired_pose, gazebo::math::Vector3 current_linvel, gazebo::math::Vector3 desired_linvel, gazebo::math::Vector3 current_angvel, gazebo::math::Vector3 desired_angvel ) {
  gazebo::math::Vector3 x = current_pose.pos;
  gazebo::math::Vector3 x_star = desired_pose.pos;
  gazebo::math::Vector3 xd = current_linvel;
  gazebo::math::Vector3 xd_star = desired_linvel;
  gazebo::math::Quaternion q = current_pose.rot;
  gazebo::math::Quaternion q_star = desired_pose.rot;

  gazebo::math::Vector3 thetad = current_angvel;
  gazebo::math::Vector3 thetad_star = desired_angvel;

/*
  std::cout << "m:" << m << std::endl;
  std::cout << "I:" << I << std::endl;
  std::cout << "dt:" << dt << std::endl;
  std::cout << "x:" << x << std::endl;
  std::cout << "x_star:" << x_star << std::endl;
  std::cout << "xd:" << xd << std::endl;
  std::cout << "xd_star:" << xd_star << std::endl;
  std::cout << "q:" << q << std::endl;
  std::cout << "q_star:" << q_star << std::endl;
  std::cout << "thetad:" << thetad << std::endl;
  std::cout << "thetad_star:" << thetad_star << std::endl;
*/

  gazebo::math::Vector3 v;
  gazebo::math::Vector3 omega;

  gazebo::math::Quaternion qd = (q_star - q) * (1.0 / dt);
//  std::cout << "qd:" << qd << std::endl;

  // assert qd not normalized
  const double EPSILON = 1e8;
  double mag = qd.x * qd.x + qd.y * qd.y + qd.z * qd.z + qd.w * qd.w;
  //std::cout << "mag:" << mag << std::endl;
  //assert( fabs( mag - 1.0 ) > EPSILON );

  v = (x_star - x) / dt + (xd_star - xd);
//  std::cout << "v:" << v << std::endl;
  omega = to_omega( q, qd ) + (thetad_star - thetad);
//  std::cout << "omega:" << omega << std::endl;

  //                 linear          +        angular
  double KE = (0.5 * v.Dot( v ) * m) + (0.5 * omega.Dot( I * omega ));
//  std::cout << "KE:" << KE << std::endl;
  return KE;
}

//-----------------------------------------------------------------------------

