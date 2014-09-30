#include "ur10_schunk_controller.h"

#include <sstream>

#define MAX_SIM_TIME 100.0

//-----------------------------------------------------------------------------

GZ_REGISTER_MODEL_PLUGIN( ur10_schunk_controller_c )


//-----------------------------------------------------------------------------
ur10_schunk_controller_c::ur10_schunk_controller_c( void ) { 

}

//-----------------------------------------------------------------------------
ur10_schunk_controller_c::~ur10_schunk_controller_c( void ) {
    gazebo::event::Events::DisconnectWorldUpdateBegin( _updateConnection );
}

//-----------------------------------------------------------------------------
void ur10_schunk_controller_c::Load( gazebo::physics::ModelPtr model, sdf::ElementPtr sdf ) {

  // -- PRINCIPLE REFERENCES --
  _world = model->GetWorld();
  _model = model;

  // -- COMPONENT REFERENCES --
  // ---- UR10 ----
  // link references
  _base = model->GetLink("base_link");
  if( !_base ) {
    gzerr << "Unable to find link: base_link\nPlugin failed to load\n";
    return;
  }

  _shoulder = model->GetLink("shoulder_link");
  if( !_shoulder ) {
    gzerr << "Unable to find link: shoulder_link\nPlugin failed to load\n";
    return;
  }

  _upperarm = model->GetLink("upper_arm_link");
  if( !_upperarm ) {
    gzerr << "Unable to find link: upper_arm_link\nPlugin failed to load\n";
    return;
  }

  _forearm = model->GetLink("forearm_link");
  if( !_forearm ) {
    gzerr << "Unable to find link: forearm_link\nPlugin failed to load\n";
    return;
  }

  _wrist1 = model->GetLink("wrist_1_link");
  if( !_wrist1 ) {
    gzerr << "Unable to find link: wrist_1_link\nPlugin failed to load\n";
    return;
  }

  _wrist2 = model->GetLink("wrist_2_link");
  if( !_wrist2 ) {
    gzerr << "Unable to find link: wrist_2_link\nPlugin failed to load\n";
    return;
  }

  _wrist3 = model->GetLink("wrist_3_link");
  if( !_wrist3 ) {
    gzerr << "Unable to find link: wrist_3_link\nPlugin failed to load\n";
    return;
  }

  // joint references
  _shoulder_pan_actuator = model->GetJoint("shoulder_pan_joint");
  if( !_shoulder_pan_actuator ) {
    gzerr << "Unable to find joint: shoulder_pan_joint\nPlugin failed to load\n";
    return;
  } 

  _shoulder_lift_actuator = model->GetJoint("shoulder_lift_joint");
  if( !_shoulder_lift_actuator ) {
    gzerr << "Unable to find joint: shoulder_lift_joint\nPlugin failed to load\n";
    return;
  } 

  _elbow_actuator = model->GetJoint("elbow_joint");
  if( !_elbow_actuator ) {
    gzerr << "Unable to find joint: elbow_joint\nPlugin failed to load\n";
    return;
  } 

  _wrist1_actuator = model->GetJoint("wrist_1_joint");
  if( !_wrist1_actuator ) {
    gzerr << "Unable to find joint: wrist_1_joint\nPlugin failed to load\n";
    return;
  } 

  _wrist2_actuator = model->GetJoint("wrist_2_joint");
  if( !_wrist2_actuator ) {
    gzerr << "Unable to find joint: wrist_2_joint\nPlugin failed to load\n";
    return;
  } 

  _wrist3_actuator = model->GetJoint("wrist_3_joint");
  if( !_wrist3_actuator ) {
    gzerr << "Unable to find joint: wrist_3_joint\nPlugin failed to load\n";
    return;
  } 

  // ---- SCHUNK ----
  // link references
  _hand = model->GetLink("hand");
  if( !_hand ) {
    gzerr << "Unable to find link: hand\nPlugin failed to load\n";
    return;
  }

  _finger_l = model->GetLink("l_finger");
  if( !_finger_l ) {
    gzerr << "Unable to find link: l_finger\nPlugin failed to load\n";
    return;
  }

  _finger_r = model->GetLink("r_finger");
  if( !_finger_r ) {
    gzerr << "Unable to find link: r_finger\nPlugin failed to load\n";
    return;
  }

  // joint references
  _finger_actuator_l = model->GetJoint("l_finger_actuator");
  if( !_finger_actuator_l ) {
    gzerr << "Unable to find joint: l_finger_actuator\nPlugin failed to load\n";
    return;
  } 

  _finger_actuator_r = model->GetJoint("r_finger_actuator");
  if( !_finger_actuator_r ) {
    gzerr << "Unable to find joint: r_finger_actuator\nPlugin failed to load\n";
    return;
  }

  _target_attached = false;
  if( sdf->HasElement("target_attached") ) {
    _target_attached = sdf->GetElement( "target_attached" )->GetValueBool();
  } 

  if( !_target_attached ) {
    // target references
    _target = _world->GetModel( "block" );
    if( !_target ) {
      gzerr << "Unable to find target model\nPlugin failed to load\n";
      return;
    }
  }

  // reset the world before we begin
  _world->Reset();  

  // compute the base slip value
  //_base_slip = compute_slip();

  // set the starting velocity for the joints
  const double PERIOD = 5.0;
  const double AMP = 0.5;
  const double SMALL_AMP = AMP*0.1;
  _shoulder_pan_actuator->SetVelocity(0, std::cos(0)*AMP*PERIOD);
  _shoulder_lift_actuator->SetVelocity(0, std::cos(0)*SMALL_AMP*PERIOD*2.0);
  _elbow_actuator->SetVelocity(0, std::cos(0)*AMP*PERIOD*2.0/3.0);
  _wrist1_actuator->SetVelocity(0, std::cos(0)*AMP*PERIOD*1.0/7.0);
  _wrist2_actuator->SetVelocity(0, std::cos(0)*AMP*PERIOD*2.0/11.0);
  _wrist3_actuator->SetVelocity(0, std::cos(0)*AMP*PERIOD*3.0/13.0);

  // -- CALLBACKS --
  _updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind( &ur10_schunk_controller_c::Update, this ) );

  // Get targets
  gazebo::math::Vector3 c_v, c_omega;
  // left gripper energy constants
  c_v = _finger_l->GetWorldPose().pos - _target->GetWorldPose().pos;
  _target_c_v_l = c_v;
  c_omega = to_omega( _finger_l->GetWorldPose().rot, _target->GetWorldPose().rot );
  _target_c_omega_l = c_omega;
  // right gripper energy constants
  c_v = _finger_r->GetWorldPose().pos - _target->GetWorldPose().pos;
  _target_c_v_r = c_v;
  c_omega = to_omega( _finger_r->GetWorldPose().rot, _target->GetWorldPose().rot );
  _target_c_omega_r = c_omega;
  printf( "found block target\n" );


  // -- LOGGING --
/*
  std::string engine_name = _world->GetPhysicsEngine()->GetType();
  std::string world_name = _world->GetName();
  std::stringstream ss_logname;
  ss_logname << engine_name << "_" << world_name << ".log";
*/
  std::string engine_name = _world->GetPhysicsEngine()->GetType();
  std::string world_name = _world->GetName();
  std::stringstream ss_logname;
  ss_logname << engine_name << "-" << world_name << ".log";

  energy_log = boost::shared_ptr<log_c>( new log_c( ss_logname.str() ) );
  energy_log->open();

//  slip_log = boost::shared_ptr<log_c>( new log_c( ss_logname.str() ) );
//  slip_log->open();
  //slip_log->write( "test" );
  //slip_log->close();

  //std::cout << ss_data.str() << std::endl;

  previous_t = 0.0;

  // -- FIN --
  printf( "ur10_schunk_controller has initialized\n" );
}

//-----------------------------------------------------------------------------
void ur10_schunk_controller_c::Update( ) {

  // get the current time
  double t = _world->GetSimTime().Double();
  double dt = t - previous_t;
  double real_time = _world->GetRealTime().Double(); 

  std::stringstream data;
  std::vector<double> avgKEs;

  //std::cout << "t : " << t << std::endl;
  data << t << "," << real_time;
  // compute the energy of the target
  gazebo::physics::ModelPtr target = _target;
  gazebo::physics::LinkPtr body = target->GetLink("body");
  assert( body );

  // get the inertial properties of the box
  gazebo::physics::InertialPtr gz_inertial = body->GetInertial();
  double m = gz_inertial->GetMass();
  gazebo::math::Vector3 pm = gz_inertial->GetPrincipalMoments();
  gazebo::math::Matrix3 I( pm.x, 0.0, 0.0, 0.0, pm.y, 0.0, 0.0, 0.0, pm.z);

  double KE_l = energy( _finger_l, target, _target_c_v_l, _target_c_omega_l, m, I, dt );
  double KE_r = energy( _finger_r, target, _target_c_v_r, _target_c_omega_r, m, I, dt );

  double avg_KE = (KE_l + KE_r) / 2.0;
  //avgKEs.push_back( avg_KE );

  //std::cout << i << " KE[l,r,avg],pos : " << KE_l<< "," << KE_r << "," << avg_KE << "," << target->GetWorldPose().pos << std::endl;
  data << "," << "0" << "," << KE_l << "," << KE_r << "," << avg_KE << "," << target->GetWorldPose().pos;
  
  //std::cout << "--------------------" << std::endl;
  data << std::endl;
  energy_log->write( data.str() );

  // check for exit condition
  if( fabs(avg_KE) > 1.0e7 ) {
    printf( "Kinetic Energy of the block exceeded 1e7... Killing sim.\n" );
    exit( 0 );
  }
  if( t >= MAX_SIM_TIME ) {
    printf( "Maximum simulation time exceeded... Killing sim\n" );
    exit( 0 );
  }
  


/*
  double slip_distance = compute_slip();

  std::stringstream ss_slipdata;
  ss_slipdata << t << "," << slip_distance << std::endl;
  slip_log->write( ss_slipdata.str() );
  if( slip_distance > _base_slip * 1.25 ) {
    printf( "block has slipped\n" );
    slip_log->close();
    exit( 0 );
  }
*/

  // determinet the desired position and velocity for the controller 
  const double PERIOD = 5.0;
  const double AMP = 0.5;
  const double SMALL_AMP = AMP*0.1;
  double sh_pan_q_des = std::sin(t)*AMP*PERIOD;
  double sh_pan_qd_des = std::cos(t)*AMP*PERIOD;
  double sh_lift_q_des = std::sin(t*2.0)*SMALL_AMP*PERIOD*2.0;
  double sh_lift_qd_des = std::cos(t*2.0)*SMALL_AMP*PERIOD*2.0;
  double elbow_q_des = std::sin(t*2.0/3.0)*AMP*PERIOD*2.0/3.0;
  double elbow_qd_des = std::cos(t*2.0/3.0)*AMP*PERIOD*2.0/3.0;
  double wrist1_q_des = std::sin(t*1.0/7.0)*AMP*PERIOD*1.0/7.0;
  double wrist1_qd_des = std::cos(t*1.0/7.0)*AMP*PERIOD*1.0/7.0;
  double wrist2_q_des = std::sin(t*2.0/11.0)*AMP*PERIOD*2.0/11.0;
  double wrist2_qd_des = std::cos(t*2.0/11.0)*AMP*PERIOD*2.0/11.0;
  double wrist3_q_des = std::sin(t*3.0/13.0)*AMP*PERIOD*3.0/13.0;
  double wrist3_qd_des = std::cos(t*3.0/13.0)*AMP*PERIOD*3.0/13.0;

  // compute the errors
  double sh_pan_q_err = (sh_pan_q_des - _shoulder_pan_actuator->GetAngle(0).Radian());
  double sh_pan_qd_err = (sh_pan_qd_des - _shoulder_pan_actuator->GetVelocity(0));
  double sh_lift_q_err = (sh_lift_q_des - _shoulder_lift_actuator->GetAngle(0).Radian());
  double sh_lift_qd_err = (sh_lift_qd_des - _shoulder_lift_actuator->GetVelocity(0));
  double elbow_q_err = (elbow_q_des - _elbow_actuator->GetAngle(0).Radian());
  double elbow_qd_err = (elbow_qd_des - _elbow_actuator->GetVelocity(0));
  double wrist1_q_err = (wrist1_q_des - _wrist1_actuator->GetAngle(0).Radian());
  double wrist1_qd_err = (wrist1_qd_des - _wrist1_actuator->GetVelocity(0));
  double wrist2_q_err = (wrist2_q_des - _wrist2_actuator->GetAngle(0).Radian());
  double wrist2_qd_err = (wrist2_qd_des - _wrist2_actuator->GetVelocity(0));
  double wrist3_q_err = (wrist3_q_des - _wrist3_actuator->GetAngle(0).Radian());
  double wrist3_qd_err = (wrist3_qd_des - _wrist3_actuator->GetVelocity(0));

  // setup gains
  const double SH_KP = 300.0, SH_KV = 120.0;
  const double EL_KP = 60.0, EL_KV = 24.0;
  const double WR_KP = 15.0, WR_KV = 6.0;
 
  // compute the actuator forces
  double sh_pan_f = SH_KP*sh_pan_q_err + SH_KV*sh_pan_qd_err;
  double sh_lift_f = SH_KP*sh_lift_q_err + SH_KV*sh_lift_qd_err;
  double elbow_f = EL_KP*elbow_q_err + EL_KV*elbow_qd_err;
  double wrist1_f = WR_KP*wrist1_q_err + WR_KV*wrist1_qd_err;
  double wrist2_f = WR_KP*wrist2_q_err + WR_KV*wrist2_qd_err;
  double wrist3_f = WR_KP*wrist3_q_err + WR_KV*wrist3_qd_err;

  // set the actuator forces for the arm
  _shoulder_pan_actuator->SetForce(0, sh_pan_f);
  _shoulder_lift_actuator->SetForce(0, sh_lift_f);
  _elbow_actuator->SetForce(0, elbow_f);
  _wrist1_actuator->SetForce(0, wrist1_f);
  _wrist2_actuator->SetForce(0, wrist2_f);
  _wrist3_actuator->SetForce(0, wrist3_f);

  // Simple close fingers test case
  _finger_actuator_l->SetForce(0, 1);  // close
  //_finger_actuator_l->SetForce(0, -0.00001);  // open

  _finger_actuator_r->SetForce(0,-1); // close
  //_finger_actuator_r->SetForce(0,0.00001);  //open

  previous_t = t;
}

//-----------------------------------------------------------------------------
/*
void ur10_schunk_controller_c::Reset( ) {

}
*/
//-----------------------------------------------------------------------------
double ur10_schunk_controller_c::compute_slip( void ) {
  if( !_target_attached ) {
    gazebo::math::Vector3 slip_vector;
    slip_vector = _target->GetWorldPose().pos - _hand->GetWorldCoGPose().pos;
    return slip_vector.GetLength();
  }

  return 1;
}

//-----------------------------------------------------------------------------
gazebo::math::Vector3 ur10_schunk_controller_c::to_omega( gazebo::math::Quaternion q, gazebo::math::Quaternion qd ) {
  gazebo::math::Vector3 omega;
  omega.x = 2 * (-q.x * qd.w + q.w * qd.x - q.z * qd.y + q.y * qd.z);
  omega.y = 2 * (-q.y * qd.w + q.z * qd.x + q.w * qd.y - q.x * qd.z);
  omega.z = 2 * (-q.z * qd.w - q.y * qd.x + q.x * qd.y + q.w * qd.z);
  return omega;

}

//-----------------------------------------------------------------------------
gazebo::math::Quaternion ur10_schunk_controller_c::deriv(gazebo::math::Quaternion q, gazebo::math::Vector3 w) {
  gazebo::math::Quaternion qd;

  qd.w = .5 * (-q.x * w.x - q.y * w.y - q.z * w.z); 
  qd.x = .5 * (+q.w * w.x + q.z * w.y - q.y * w.z);
  qd.y = .5 * (-q.z * w.x + q.w * w.y + q.x * w.z);
  qd.z = .5 * (+q.y * w.x - q.x * w.y + q.w * w.z);

  return qd;
}

//-----------------------------------------------------------------------------
double ur10_schunk_controller_c::energy( gazebo::physics::LinkPtr gripper, gazebo::physics::ModelPtr grip_target, gazebo::math::Vector3 c_v, gazebo::math::Vector3 c_omega, double m, gazebo::math::Matrix3 I, double dt ) {

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
double ur10_schunk_controller_c::energy( double m, gazebo::math::Matrix3 I, double dt, gazebo::math::Pose current_pose, gazebo::math::Pose desired_pose, gazebo::math::Vector3 current_linvel, gazebo::math::Vector3 desired_linvel, gazebo::math::Vector3 current_angvel, gazebo::math::Vector3 desired_angvel ) {
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
