/* Copyright [2016] [Alessandro Settimi (ale.settimi@gmail.com), Mirko Ferrati, Danilo Caporale, Edoardo Farnioli]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

#include <yarp/os/all.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <assert.h>
#include <iostream>
#include <iCub/iDynTree/yarp_kdl.h>
#include "multicontact_module_thread.h"
#include <kdl/frames_io.hpp>
#include <idynutils/RobotUtils.h>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#define DEG2RAD M_PI/180.0 //to be multiplied

using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using namespace walkman;

std::string green(std::string text)
{
	std::string g="\033[0;32m";
	std::string b="\033[0m";
	return g + text + b;
}

std::string yellow(std::string text)
{
	std::string y="\033[0;33m";
	std::string b="\033[0m";
	return y + text + b;
}

std::string red(std::string text)
{
	std::string r="\033[0;31m";
	std::string b="\033[0m";
	return r + text + b;
}

std::string cyan(std::string text)
{
	std::string c="\033[0;36m";
	std::string b="\033[0m";
	return c + text + b;
}

multicontact_thread::multicontact_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr< paramHelp::ParamHelperServer > ph):
	control_thread( module_prefix, rf, ph ), recv_interface("multicontact_interface"), ack_interface("multicontact_ack"),
	IK(get_robot_name(),get_urdf_path(),get_srdf_path(),get_thread_period()),
// size of q vectors
	size_q(locoman::utils::getNumberOfKinematicJoints(robot)),
	q_sense(size_q,0.0),
	q_current(size_q, 0.0),
	q_offset(size_q, 0.0),   // q_current = q_sense + q_offset
	q_des(size_q, 0.0),
	d_q_move(size_q, 0.0),
// for FT_sensors filtering
	WINDOW_size(5),
	SENSORS_WINDOW(24,WINDOW_size),
	SENSORS_SUM(24, 0.0), 
	SENSORS_FILTERED(24, 0.0),
	SENSORS_AT_COMMAND(24,0.0),
	ft_l_ankle(6,0.0),
	ft_r_ankle(6,0.0),
	ft_l_wrist(6,0.0),
	ft_r_wrist(6,0.0),
	Sensor_Collection(24,0.0),

	fc_l_c_to_world(12,0.0),
	fc_r_c_to_world(12,0.0) ,
	fc_feet_to_world(24,0.0) ,

	fc_l_c_hand_to_world(12,0.0)  ,
	fc_r_c_hand_to_world(12,0.0)  ,
	fc_hand_to_world(24,0.0) ,
// for contact force vector calculation
	map_l_fcToSens_PINV(12,6) ,
	map_r_fcToSens_PINV(12,6) ,
	map_l_hand_fcToSens_PINV(12,6) ,
	map_r_hand_fcToSens_PINV(12,6) ,

	fc_offset_left(12,0.0) ,
	fc_offset_right(12,0.0) ,

	fc_offset_left_hand(12,0.0) ,
	fc_offset_right_hand(12,0.0) ,

	fc_current_left(12,0.0) ,//= fc_sense_left  - fc_offset_left ,
	fc_current_right(12,0.0) ,//= fc_sense_right - fc_offset_right , 
	fc_current_left_hand(12,0.0) ,// = fc_sense_left_hand  - fc_offset_left_hand  , 
	fc_current_right_hand(12,0.0) ,//= fc_sense_right_hand - fc_offset_right_hand  ,  

	FC_to_world(48,0.0) ,

	fc_sense_left(12,0.0),
	fc_sense_right(12,0.0),
	fc_sense_left_hand(12,0.0),
	fc_sense_right_hand(12,0.0),
	
	FC_DES_NO_RIGHT_FOOT(48,0.0),
	FC_DES_NO_LEFT_FOOT(48,0.0),
        FC_DES_feet(24, 0.0)  ,
        FC_DES_hands(24,0.0) ,
        d_fc_total_des(48,0.0) ,
	
        Big_Rf_new(48, 37) 
{
state_map.emplace("wholebody_ik",false)  ;
state_map.emplace("touch",false)  ;
state_map.emplace("force",false)  ;
  
    input.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    output.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    home.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    q_init.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
	q_desired.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    delta.resize(model.iDyn3_model.getNrOfDOFs(),0.0);

    yarp::sig::Vector q_right_arm(7,0.0);
    yarp::sig::Vector q_left_arm(7,0.0);
    yarp::sig::Vector q_torso(3,0.0);
    yarp::sig::Vector q_right_leg(6,0.0);
    yarp::sig::Vector q_left_leg(7,0.0);
    yarp::sig::Vector q_head(2,0.0);

    q_head[0] = 0.0;
    q_head[1] = 0.0;

    // 59.894769 -9.56636 20.572535 -109.180362 -61.110078 -28.820571 -1.040955
    q_right_arm[0]= 60*DEG2RAD;
    q_right_arm[1]= -10*DEG2RAD;
    q_right_arm[2]= 20*DEG2RAD;
    q_right_arm[3]= -110*DEG2RAD;
    q_right_arm[4]= -60*DEG2RAD;
    q_right_arm[5]= -30*DEG2RAD;
    // 59.822628 9.731391 -20.199733 -109.027161 60.439224 -29.08287 1.049194
    q_left_arm[0]= 60*DEG2RAD;
    q_left_arm[1]= 10*DEG2RAD;
    q_left_arm[2]= -20*DEG2RAD;
    q_left_arm[3]= -110*DEG2RAD;
    q_left_arm[4]= 60*DEG2RAD;
    q_left_arm[5]= -30*DEG2RAD;

    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
    // 1.888115 0.748648 -15.351364 23.773366 -10.643065 -2.006551
    q_right_leg[0]= 2*DEG2RAD;
    q_right_leg[2]= -15*DEG2RAD;
    q_right_leg[3]=  25*DEG2RAD;
    q_right_leg[4]= -10*DEG2RAD;
    q_right_leg[5]= -2*DEG2RAD;
    // -3.044729 0.428668 -14.159493 23.95687 -10.984182 2.724435
    q_left_leg[0]= -2*DEG2RAD;
    q_left_leg[2]= -15*DEG2RAD;
    q_left_leg[3]= 25*DEG2RAD;
    q_left_leg[4]= -10*DEG2RAD;
    q_left_leg[5]= 2*DEG2RAD;
    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,home);

    // populate command list
//     wb_cmd.add_command("idle");

	setup_wb_ik();
}

bool multicontact_thread::custom_init()
{
    log_sense.start("q_sense.txt");
    log_1.start("q_output_1.txt");
    log_2.start("q_delta_1.txt");
    log_3.start("q_delta_2.txt");
    log_4.start("q_output_2.txt");

    run_counter = 0 ;
    //  real time thread
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam ( pthread_self(), SCHED_FIFO, &thread_param );

    // setup
    model.iDyn3_model.setFloatingBaseLink(model.left_leg.end_effector_index);
    sense();
    input = robot.sensePosition();
	input[model.iDyn3_model.getDOFIndex("WaistLat")] = 0.0;
	output = input;

    q_init = input;
    
    robot.setPositionDirectMode();

    // initial position
    // go_in_initial_position();

   //--------------------------------------------------------------------------------------------------------------
   // YARP Port Section - SEND
   if(!sending_q.open(std::string("/" + get_module_prefix() + "/sending_q"))) {
        std::cout << "ERROR: cannot open YARP port " << std::string(get_module_prefix() + "/sending_q") << std::endl;
        return false; } 
   yarp::os::Network::connect( std::string("/" + get_module_prefix() + "/sending_q"), "/locoman_service_2/receiving_q");       
    
   if(!sending_fc.open(std::string("/" + get_module_prefix() + "/sending_fc"))) {
        std::cout << "ERROR: cannot open YARP port " << std::string(get_module_prefix() + "/sending_fc") << std::endl;
        return false; } 
   yarp::os::Network::connect( std::string("/" + get_module_prefix() + "/sending_fc"), "/locoman_service_2/receiving_fc");       
   // end of the ...  YARP Port Section -SEND
   //----------------------------------------------------------------------------------------------------------------
   
  if(!receiving_Big_Rf.open(std::string("/" + get_module_prefix() + "/receiving_Big_Rf"))) {
  std::cout << "ERROR: cannot open YARP port " << std::string(get_module_prefix() + "/receiving_Big_Rf") << std::endl;
  return false;  }
  if(!yarp::os::Network::connect(  "/locoman_service_2/sending_Big_Rf", std::string("/" + get_module_prefix() + "/receiving_Big_Rf"))){
  std::cout << "ERROR connecting YARP ports " << std::endl ;
  return false ;
  }   
  receiving_Big_Rf_initted = false ;
   //----------------------------------------------------------

  // Computation PINV for FT sensor processing
    waist_index   = model.iDyn3_model.getLinkIndex("Waist");
    l_ankle_index = model.iDyn3_model.getLinkIndex("l_leg_ft") ;
    //     int l_ankle_index = model.iDyn3_model.getLinkIndex("l_foot_upper_left_link") ;
    l_c1_index    = model.iDyn3_model.getLinkIndex("l_foot_upper_left_link") ;
    l_c2_index    = model.iDyn3_model.getLinkIndex("l_foot_upper_right_link");
    l_c3_index    = model.iDyn3_model.getLinkIndex("l_foot_lower_left_link") ;
    l_c4_index    = model.iDyn3_model.getLinkIndex("l_foot_lower_right_link");

    r_ankle_index = model.iDyn3_model.getLinkIndex("r_leg_ft") ;
    //     int r_ankle_index = model.iDyn3_model.getLinkIndex("r_foot_upper_left_link") ;
    r_c1_index    = model.iDyn3_model.getLinkIndex("r_foot_upper_left_link");
    r_c2_index    = model.iDyn3_model.getLinkIndex("r_foot_upper_right_link");
    r_c3_index    = model.iDyn3_model.getLinkIndex("r_foot_lower_left_link");
    r_c4_index    = model.iDyn3_model.getLinkIndex("r_foot_lower_right_link");

    l_hand_index  = model.iDyn3_model.getLinkIndex("LSoftHand");
    r_hand_index  = model.iDyn3_model.getLinkIndex("RSoftHand");

    l_wrist_index  = model.iDyn3_model.getLinkIndex("l_arm_ft") ;
    l_hand_c1_index = model.iDyn3_model.getLinkIndex("l_hand_upper_right_link");  // r_foot_upper_left_link
    l_hand_c2_index = model.iDyn3_model.getLinkIndex("l_hand_lower_right_link");  // r_foot_upper_right_link
    l_hand_c3_index = model.iDyn3_model.getLinkIndex("l_hand_upper_left_link");   // r_foot_lower_left_link
    l_hand_c4_index = model.iDyn3_model.getLinkIndex("l_hand_lower_left_link");  // r_foot_lower_right_link
    //
    r_wrist_index   = model.iDyn3_model.getLinkIndex("r_arm_ft") ;
    r_hand_c1_index = model.iDyn3_model.getLinkIndex("r_hand_upper_right_link");  // r_foot_upper_left_link
    r_hand_c2_index = model.iDyn3_model.getLinkIndex("r_hand_lower_right_link");  // r_foot_upper_right_link
    r_hand_c3_index = model.iDyn3_model.getLinkIndex("r_hand_upper_left_link");   // r_foot_lower_left_link
    r_hand_c4_index = model.iDyn3_model.getLinkIndex("r_hand_lower_left_link");  // r_foot_lower_right_link

    map_l_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( locoman::utils::fConToSens( l_ankle_index, 
																					  l_c1_index  , 
																					  l_c2_index  ,
																					  l_c3_index  , 
																					  l_c4_index,
																					  model
															  ) , 1E-10 ) ;
    map_r_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( locoman::utils::fConToSens( r_ankle_index, 
																					  r_c1_index, 
																					  r_c2_index,
																					  r_c3_index, 
																					  r_c4_index,
																					  model
															  ), 1E-10 ) ; 
    map_l_hand_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( locoman::utils::fConToSens( l_wrist_index, 
																						   l_hand_c1_index, 
																						   l_hand_c2_index,
																						   l_hand_c3_index, 
																						   l_hand_c4_index,
																						   model
																   ), 1E-10 ) ; 
    map_r_hand_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( locoman::utils::fConToSens( r_wrist_index, 
																						   r_hand_c1_index, 
																						   r_hand_c2_index,
																						   r_hand_c3_index, 
																						   r_hand_c4_index ,
																						   model
																   ), 1E-10 ) ; 
    // sensor force biasing
    //-----------------------------------------------------------
    // Evaluating offset for contact forces
	//------------------------------------------------------------------------------------------
	char vai_01 ;
	std::cout << " Press 'y' if you need to compute the sensors offset !!! Otherwise press any key !!!  " << std::endl ;
	//std::cout << " waiting for a keyboard input !!! " << std::endl ;
	std::cin >> vai_01 ;
	//-------------------
	fc_offset_left.zero()  ;
	fc_offset_right.zero() ;
	fc_offset_left_hand.zero() ;
	fc_offset_right_hand.zero() ;
	
	if (vai_01 == 'y' ) {
       int dim_fc_offset = 1000 ;  

	for(int k=0; k<dim_fc_offset ; k++ ){
		robot.senseftSensor("l_leg_ft", ft_l_ankle) ;
		robot.senseftSensor("r_leg_ft", ft_r_ankle) ;
		robot.senseftSensor("l_arm_ft", ft_l_wrist) ;
		robot.senseftSensor("r_arm_ft", ft_r_wrist) ;  
		fc_offset_left  += map_l_fcToSens_PINV * ft_l_ankle ;
		fc_offset_right += map_r_fcToSens_PINV * ft_r_ankle ;  
		fc_offset_left_hand  += map_l_hand_fcToSens_PINV * ft_l_wrist  ;
		fc_offset_right_hand += map_r_hand_fcToSens_PINV * ft_r_wrist  ; 
		usleep(1*1000) ; 
	}

	fc_offset_left       = fc_offset_left/ dim_fc_offset ;
	fc_offset_right      = fc_offset_right/ dim_fc_offset ;
	fc_offset_left_hand  = fc_offset_left_hand/ dim_fc_offset ;
	fc_offset_right_hand = fc_offset_right_hand/ dim_fc_offset ;
  }
	// first measures setup and cout
    
	fc_sense_left  = map_l_fcToSens_PINV * ft_l_ankle ;
	fc_sense_right = map_r_fcToSens_PINV * ft_r_ankle ; 
	fc_sense_left_hand  = map_l_hand_fcToSens_PINV * ft_l_wrist  ; 
	fc_sense_right_hand = map_r_hand_fcToSens_PINV * ft_r_wrist  ; 

	fc_current_left  = fc_sense_left  - fc_offset_left ;
	fc_current_right = fc_sense_right - fc_offset_right ; 
	fc_current_left_hand  = fc_sense_left_hand  - fc_offset_left_hand  ; 
	fc_current_right_hand = fc_sense_right_hand - fc_offset_right_hand  ;  

	std::cout << " fc_sense_left = "  << fc_sense_left.toString() << std::endl;
	std::cout << " fc_sense_right = " << fc_sense_right.toString() << std::endl;
	std::cout << " fc_sense_left_hand = "  << fc_sense_left_hand.toString() << std::endl;
	std::cout << " fc_sense_right_hand = " << fc_sense_right_hand.toString() << std::endl;

	std::cout << " fc_offset_left = "  << fc_offset_left.toString() << std::endl;
	std::cout << " fc_offset_right = " << fc_offset_right.toString() << std::endl;
	std::cout << " fc_offset_left_hand = "  << fc_offset_left_hand.toString() << std::endl;
	std::cout << " fc_offset_right_hand = " << fc_offset_right_hand.toString() << std::endl;

	std::cout << " fc_current_left = "  << fc_current_left.toString() << std::endl;
	std::cout << " fc_current_right = " << fc_current_right.toString() << std::endl;
	std::cout << " fc_current_left_hand = "  << fc_current_left_hand.toString() << std::endl;
	std::cout << " fc_current_right_hand = " << fc_current_right_hand.toString() << std::endl;

	//------------------------------------------------------------------------------------------
	char vai_1 ;
	std::cout << " Put the Robot DOWN on the terrain and press a key !!! " << std::endl ;
	//std::cout << " waiting for a keyboard input !!! " << std::endl ;
	std::cin >> vai_1 ;
//  //-------------------
	read_offset_q();

    // filling sensors window
	
	//Getting Force/Torque Sensor Measures
	if(!robot.senseftSensor("l_leg_ft", ft_l_ankle)) std::cout << "ERROR READING SENSOR l_ankle" << std::endl;
	if(!robot.senseftSensor("r_leg_ft", ft_r_ankle)) std::cout << "ERROR READING SENSOR r_ankle" << std::endl;
	if(!robot.senseftSensor("l_arm_ft", ft_l_wrist)) std::cout << "ERROR READING SENSOR l_wrist" << std::endl;
	if(!robot.senseftSensor("r_arm_ft", ft_r_wrist)) std::cout << "ERROR READING SENSOR r_wrist" << std::endl;

	// Filtering The Sensors
	Sensor_Collection.setSubvector(0, ft_l_ankle )   ;
	Sensor_Collection.setSubvector(6, ft_r_ankle )   ;
	Sensor_Collection.setSubvector(12, ft_l_wrist )  ;
	Sensor_Collection.setSubvector(18, ft_r_wrist )  ;

	// HACK to prevent disturbances from FT_sensors
	Sensor_Collection[0]=0.0 ;
	Sensor_Collection[1]=0.0 ;
	Sensor_Collection[6]=0.0 ;
	Sensor_Collection[7]=0.0 ;
	
	// Initializing Sensor related variables 
	for(int t=0; t<WINDOW_size ; t++ )
	{
	  SENSORS_WINDOW.setCol(t, Sensor_Collection )   ;
	}
	SENSORS_SUM = WINDOW_size * Sensor_Collection ;
	SENSORS_FILTERED = Sensor_Collection ;
	count_sensor = 0 ;

	IK.initialize("wb_left",input);
	initialized.at("wb_left")=true;
	IK.initialize("wb_right",input);
	initialized.at("wb_right")=true;
	done = true;

	std::cout<<" - " + green("Initialized")<<std::endl;

	return true;
}

void multicontact_thread::read_offset_q(){
	// offset on q
	// q_ Offset Evaluation Section
	int dim_offeset = 1000;
	yarp::sig::Matrix offset_window(locoman::utils::getNumberOfKinematicJoints(robot), dim_offeset);
	q_current.zero();
	for(int k=0; k<dim_offeset ; k++ ){
		q_current += locoman::utils::sense_position_no_hands(robot); //if sense returns motorPosition
		usleep(1*1000) ;
	}
	q_current = q_current/dim_offeset ;

	q_offset = input - q_current ;

}

void multicontact_thread::send_to_service2() {
  //---------------------------------------------------------------------------------------------------------
  // Yarp Ports: Sending Robot Configuration and Contact Forces 
     
  yarp::sig::Vector &sending_q_vect = sending_q.prepare() ; 
  sending_q_vect.resize( input.size() ) ;
  sending_q_vect = input ;
  sending_q.write() ;
//   std::cout << " q_to_service_2 = " << input.toString() << std::endl ;

  yarp::sig::Vector &sending_fc_vect = sending_fc.prepare() ;
  sending_fc_vect.resize( FC_to_world.size() ) ;
  sending_fc_vect = FC_to_world ;
  sending_fc.write() ;
//   std::cout << "FC_to_service_2 = " << FC_to_world.toString() << std::endl  ;
   
}

void multicontact_thread::read_from_service2() {
  if(!receiving_Big_Rf_initted){ Big_Rf_received = receiving_Big_Rf.read();  // if we are in the first loop... waiting for data 
      if(Big_Rf_received){ Big_Rf_data = *Big_Rf_received;
	receiving_Big_Rf_initted = true ; }
  }
  else {  Big_Rf_received = receiving_Big_Rf.read(false) ; // do not wait for data
      if(Big_Rf_received){Big_Rf_data = *Big_Rf_received; }
  };  
  
  Big_Rf_new = Big_Rf_data ;
  
//     std::cout << "Big_Rf_new.rows() = " << Big_Rf_new.rows() << std::endl ;
//     std::cout << "Big_Rf_new.cols() = " << Big_Rf_new.cols() << std::endl ;
//     std::cout << "Big_Rf_new.toString() = " << Big_Rf_new.toString() << std::endl ;    
    
}

void multicontact_thread::run()
{
//     utils_1.tic() ;
    sense();

    send_to_service2();

    read_from_service2();

    // get the command
    if(recv_interface.getCommand(msg,recv_num))
    {    
      SENSORS_AT_COMMAND = SENSORS_FILTERED;
      std::cout<<"Command received: "<<msg.command<<std::endl;

	  if (msg.command=="stop") {
		set_idle_state();
		std::cout<<"-- " + cyan(" >> STOP << ")<<std::endl;
	  }

        else if(msg.command=="reset")
        {
			set_state("wholebody_ik");
            go_in_initial_position();
            time=0;
        }
        //----------------------------------------------------------
	// WB IK Commands!
	else if (generate_poses_from_cmd() ){ 
	      set_state("wholebody_ik");
	      time=0;
	}
        //----------------------------------------------------------
	// Hybrid Force/Position Commands!
        else if (msg.command=="touch") {
		set_state("touch");
        
        generate_touch_poses();
	}
	//----------------------------------------------------------
	// Control Force Commands!
	else if (msg.command=="force_no_left"){
	      std::cout<<"force_no_left :  WE ARE IN !!!!"<<std::endl;
	      set_state("force");
	}
	else if (msg.command=="force_no_right"){
            std::cout<<"force_no_right :  WE ARE IN !!!!"<<std::endl;
	      set_state("force");
	}
	//------------------------------------------------------
        else // TODO : verify this!
        {
	  std::cout<<"Received unknown command!!!"<<std::endl;
            
        }
    }

    control_law();

//     std::cout<<run_counter<< std::endl ; 
//     run_counter++ ;
    move();
//     utils_1.toc() ;
}

void multicontact_thread::set_idle_state()
{
	for(auto& state:state_map)
		state.second=false;
}

void multicontact_thread::set_state(std::string key) {
  std::map<std::string,bool>::iterator it;
  for(it=state_map.begin(); it!=state_map.end(); ++it) {
    if(it->first==key) {
      it->second=true;
    }
    else it->second = false;
  }
}

bool multicontact_thread::is_state_active(std::string key) {
  std::map<std::string,bool>::iterator it;
  for(it=state_map.begin(); it!=state_map.end(); ++it) {
    if(it->first.find(key) != std::string::npos) {
      if(it->second == true) {
	return true;
      }
    }
  }
  return false;
}

void multicontact_thread::sense()
{
    // joint positions
    //input = output; // robot.sensePosition(); // FIXME
    model.updateiDyn3Model( input, true );
    // ft sensors
    //Getting Force/Torque Sensor Measures
    if(!robot.senseftSensor("l_leg_ft", ft_l_ankle)) std::cout << "ERROR READING SENSOR l_ankle" << std::endl;
    if(!robot.senseftSensor("r_leg_ft", ft_r_ankle)) std::cout << "ERROR READING SENSOR r_ankle" << std::endl;
    if(!robot.senseftSensor("l_arm_ft", ft_l_wrist)) std::cout << "ERROR READING SENSOR l_wrist" << std::endl;
    if(!robot.senseftSensor("r_arm_ft", ft_r_wrist)) std::cout << "ERROR READING SENSOR r_wrist" << std::endl;

    // Filtering The Sensors
    Sensor_Collection.setSubvector(0, ft_l_ankle )   ;
    Sensor_Collection.setSubvector(6, ft_r_ankle )   ;
    Sensor_Collection.setSubvector(12, ft_l_wrist )  ;
    Sensor_Collection.setSubvector(18, ft_r_wrist )  ;

    // HACK to prevent disturbances from FT_sensors
    Sensor_Collection[0]=0.0 ;
    Sensor_Collection[1]=0.0 ;
    Sensor_Collection[6]=0.0 ;
    Sensor_Collection[7]=0.0 ;

    count_sensor = count_sensor% WINDOW_size ;
    SENSORS_WINDOW.setCol( count_sensor , Sensor_Collection ) ;
    SENSORS_SUM = SENSORS_SUM + Sensor_Collection -1.0 * SENSORS_WINDOW.getCol((count_sensor+ 1)%WINDOW_size);
    SENSORS_FILTERED = SENSORS_SUM / (WINDOW_size-1.0) ;
    count_sensor += 1 ;
    
    contact_force_vector_computation() ;

	broadcast_com_tf();
}

void multicontact_thread::contact_force_vector_computation() {
	//---------------------------------------------------------------------------------------------------------
	// Contact Force Vector Computation Section

	// FEET
	fc_l_c_to_world = map_l_fcToSens_PINV * SENSORS_FILTERED.subVector(  0,5  ) ; //ft_l_ankle  ;  // yarp::math::pinv( map_l_fcToSens, 1E-6)  *  ft_l_ankle     ;
	fc_r_c_to_world = map_r_fcToSens_PINV * SENSORS_FILTERED.subVector(  6,11 ) ; //ft_r_ankle  ;    

	if(  !(flag_robot ==1  && robot.idynutils.getRobotName() == "bigman")  ){  // Changing the sign again if we are not on the walkman real robot
		fc_l_c_to_world  = -1.0*fc_l_c_to_world ;                  // in the walkman (real) robot the feet sensors provide to_wolrd measures
		fc_r_c_to_world  = -1.0*fc_r_c_to_world ; 
	}
 
	fc_current_left  = fc_l_c_to_world  - fc_offset_left   ; 
	fc_current_right = fc_r_c_to_world - fc_offset_right  ; 
  
	fc_feet_to_world.setSubvector(0, fc_current_left ) ;
	fc_feet_to_world.setSubvector(fc_current_right.length(), fc_r_c_to_world ) ; 
    
	//-------------------------------
	// HANDS
	fc_l_c_hand_to_world = map_l_hand_fcToSens_PINV * SENSORS_FILTERED.subVector( 12,17 ); //ft_l_wrist  ;  // TODO :  verifica segno su sim e su robot
	fc_r_c_hand_to_world = map_r_hand_fcToSens_PINV * SENSORS_FILTERED.subVector( 18,23 ); //ft_r_wrist  ;  // yarp::math::pinv( map_r_fcToSens, 1E-6)  *  ft_r_ankle     ;
  
	if(  !(flag_robot ==1  && robot.idynutils.getRobotName() == "bigman")  ){  // Changing the sign again if we are not on the walkman real robot
		fc_l_c_hand_to_world  = -1.0*fc_l_c_hand_to_world ;                  // in the walkman (real) robot the feet sensors provide to_wolrd measures
		fc_r_c_hand_to_world  = -1.0*fc_r_c_hand_to_world ; 
	}
  
	fc_l_c_hand_to_world -= fc_offset_left_hand  ;
	fc_r_c_hand_to_world -= fc_offset_right_hand ;
	//
	fc_hand_to_world.setSubvector(0, fc_l_c_hand_to_world ) ;
	fc_hand_to_world.setSubvector(fc_l_c_hand_to_world.length(), fc_r_c_hand_to_world ) ;  
	//---------------------------
  
	FC_to_world.setSubvector(0, fc_feet_to_world) ;
	FC_to_world.setSubvector(fc_feet_to_world.length(), fc_hand_to_world) ;  

	// End of Contact Force Vector Computation Section
}

void multicontact_thread::control_law()
{  
  if(state_map["wholebody_ik"]) {
    control_law_ik();
  } 
  else if(state_map["touch"]) {   // BEGINNING the TOUCH part...
    sense();

    control_law_ik();

    if(msg.touch.at("l_sole")) {  // going in touch with the left foot
      
      if(!(SENSORS_FILTERED[2] - SENSORS_AT_COMMAND[2] <= -DELTA_F_MAX)) 
      {

      } 
      else 
	  {
		output = input;
		state_map["touch"] = false;
		std::cout<<"-- " + green("touch done")<<std::endl;
		ack_interface.sendCommand("touch done",cmd_num++);
      }
    } 
    else if(msg.touch.at("r_sole")) 
	{   // going in touch with the right foot
      
      if(!(SENSORS_FILTERED[8] - SENSORS_AT_COMMAND[8] <= -DELTA_F_MAX)) 
	  {

      }
      else 
	  {
		output = input;
		state_map["touch"] = false;
		std::cout<<"-- " + green("touch done")<<std::endl;
		ack_interface.sendCommand("touch done",cmd_num++);
      }
    } else {

      if(msg.touch.at("LSoftHand") && msg.touch.at("RSoftHand")) {  // going in touch with both the hands
	bool still_moving = false;

	  if(!(SENSORS_FILTERED[13] - SENSORS_AT_COMMAND[13] <= -DELTA_F_MAX)) { // if not touching yet with left hand
	    still_moving = true;

	  } 
	  if(!(SENSORS_FILTERED[19] - SENSORS_AT_COMMAND[19] >= DELTA_F_MAX)) { // if not touching yet with right hand
	    still_moving = true;

	  } 

	  if(!still_moving) {
	    state_map["touch"] = false;
		std::cout<<"-- " + green("touch done")<<std::endl;
		ack_interface.sendCommand("touch done",cmd_num++);
	  }
      } 
      else {
	if(msg.touch.at("LSoftHand")) 
	{	   // going in touch ONLY with the Left hand
	  if(!(SENSORS_FILTERED[13] - SENSORS_AT_COMMAND[13] <= -DELTA_F_MAX)) 
	  {

	  } 
	  else 
	  {
	    output = input;
	    state_map["touch"] = false;
		std::cout<<"-- " + green("touch done")<<std::endl;
		ack_interface.sendCommand("touch done",cmd_num++);
	  }
	}
	if(msg.touch.at("RSoftHand")) {  // going in touch ONLY with the Right hand
	  if(!(SENSORS_FILTERED[19] - SENSORS_AT_COMMAND[19] >= DELTA_F_MAX)) { 

	  } 
	  else {
	    output = input;
	    state_map["touch"] = false;
		std::cout<<"-- " + green("touch done")<<std::endl;
		ack_interface.sendCommand("touch done",cmd_num++);
	  }
	}
      }
    } 
  }// END of the TOUCH part 
  else if(state_map["force"]) {   // BEGINNING the FORCE part...
    if(msg.command=="force_no_right"){    
	if( SENSORS_FILTERED[8]< 30.0 )  // if the right foot sense more than 3 kg do... (task not realized!) 
	{
	mg = 1200 ;
	feet_part  = 2.0/3.0 ;
	hands_part = 1.0-feet_part ;
	mg_foot  = feet_part*mg ;
	mg_hands = hands_part*mg ;
	
	locoman::utils::FC_DES_left(   FC_DES_feet , mg_foot  ) ;
	locoman::utils::FC_DES_center_hands( FC_DES_hands, mg_hands ) ;
	FC_DES_NO_RIGHT_FOOT.setSubvector(0 , FC_DES_feet) ;
	FC_DES_NO_RIGHT_FOOT.setSubvector(FC_DES_feet.length(), FC_DES_hands) ;
	
	d_fc_total_des = FC_DES_NO_RIGHT_FOOT -1.0*FC_to_world;
	
	regu_filter = 1E9 ; 
	
	Big_Rf_new = locoman::utils::filter_k_eps_SVD( Big_Rf_new , 24, 1E-15) ;

	d_q_move = -1.0*locoman::utils::Pinv_Regularized( Big_Rf_new , regu_filter, 1E-10 )* d_fc_total_des ;
	
	output = input + d_q_move ; 
	}
	else{  // if the right foot sense less than 3 kg do nothing (task realized!)
	  output = input ;
	  state_map["force"] = false;
	  std::cout<<"-- " + green("force done")<<std::endl;
	  ack_interface.sendCommand("force done",cmd_num++);
	} 
    }
    else if (msg.command=="force_no_left"){ 
      	if( SENSORS_FILTERED[2]< 30.0 )  // if the right foot sense more than 3 kg do... (task not realized!) 
	{
	mg = 1200 ;
	feet_part  = 2.0/3.0 ;
	hands_part = 1.0-feet_part ;
	mg_foot  = feet_part*mg ;
	mg_hands = hands_part*mg ;
	
	locoman::utils::FC_DES_right(   FC_DES_feet , mg_foot  ) ;
	locoman::utils::FC_DES_center_hands( FC_DES_hands, mg_hands ) ;
	FC_DES_NO_LEFT_FOOT.setSubvector(0 , FC_DES_feet) ;
	FC_DES_NO_LEFT_FOOT.setSubvector(FC_DES_feet.length(), FC_DES_hands) ;
	
	d_fc_total_des = FC_DES_NO_LEFT_FOOT -1.0*FC_to_world;
	
	regu_filter = 1E9 ; 
	
	Big_Rf_new = locoman::utils::filter_k_eps_SVD( Big_Rf_new , 24, 1E-15) ;

	d_q_move = -1.0*locoman::utils::Pinv_Regularized( Big_Rf_new , regu_filter, 1E-10 )* d_fc_total_des ;
	
	output = input + d_q_move ; 
	}
	else {
	output = input;
	state_map["force"] = false;
	std::cout<<"-- " + green("force done")<<std::endl;
	ack_interface.sendCommand("force done",cmd_num++);
	}     
    } 
    else {
    output = input;
    } 
  }// END of the TOUCH part  
}

double abs_max(yarp::sig::Vector v)
{
    double max = -9999.0;
    for(int i=0;i<v.size();i++)
    {
        if(fabs(v[i]) > max)
            max = fabs(v[i]);
    }
    return max;
}

void multicontact_thread::move()
{
    log_1.logYarpVector(output);
    log_sense.logYarpVector(robot.sensePosition());
    // to limit joint velocity
    yarp::sig::Vector new_delta = output-input;

    log_2.logYarpVector(new_delta);

    double factor = 0.005;
//     std::cout<<norm(delta)<<std::endl;
    if(norm(new_delta) > factor)
    {
        new_delta = new_delta/norm(new_delta)*factor;
    }

    log_3.logYarpVector(new_delta);
    
    double alpha = 0.3;
    new_delta = alpha*delta + (1.0-alpha)*new_delta;
    delta = new_delta;
    output = input + delta;

//     std::cout<<abs_max(delta)<<std::endl;

    log_4.logYarpVector(output);

    robot.move(output);
    input = output; // remove this line if you are using  robot.sensePosition()
}


// Stuff fromm wholebody_ik_wb_module_thread
void multicontact_thread::control_law_ik()
{
	time = time + get_thread_period()/1000.0;

	if(going_to_initial_position)
	{
		double alpha = (time>5.0)?1:time/5.0;
		output = (1-alpha)*q_init + (alpha)*q_desired;
		if(time>5.0)
		{
			going_to_initial_position = false;
			std::cout<<"-- " + green("Ready")<<std::endl;
			IK.update_model(current_chain,input); //to update
			done=true;
		}
	}
	else if(!done)
	{
		double alpha = (time>exec_time)?1:time/exec_time;
		output = (1-alpha)*q_init + (alpha)*q_desired;
		if(time>exec_time)
		{
			std::cout<<"-- " + green("Done")<<std::endl;
			IK.update_model(current_chain,input); //to update
			done=true;
			ack_interface.sendCommand("done",cmd_num++);
		}
	}
}


bool multicontact_thread::generate_poses_from_cmd()
{
	if( std::find(available_commands.begin(), available_commands.end(), msg.command)==available_commands.end() )
	{
// 		std::cout<<" !! ERROR: command not available !!"<<std::endl;
		return false;
	}
	
	done=false;

	if(msg.command=="head_up" || msg.command=="head_down" || msg.command=="head_left" || msg.command=="head_right")
	{
		double yaw_amount = 0.0;
		double pitch_amount = 0.0;
		double disp = 0.1;

		if(msg.command=="head_up") pitch_amount = -disp;
		if(msg.command=="head_down") pitch_amount = disp;
		if(msg.command=="head_left") yaw_amount = disp;
		if(msg.command=="head_right") yaw_amount = -disp;

		time=0;
		exec_time=1.0;

		q_init = input;

		q_desired = q_init;
		q_desired[model.iDyn3_model.getDOFIndex("NeckPitchj")] = q_desired[model.iDyn3_model.getDOFIndex("NeckPitchj")] + pitch_amount;
		q_desired[model.iDyn3_model.getDOFIndex("NeckYawj")] = q_desired[model.iDyn3_model.getDOFIndex("NeckYawj")] + yaw_amount;

		return true;
	}

	if(msg.command=="torso_up" || msg.command=="torso_down" || msg.command=="torso_left" || msg.command=="torso_right" || msg.command=="torso_pitch" || msg.command=="torso_yaw")
    {
        time=0;
        exec_time=1.0;

        q_init = input;

        q_desired = q_init;
        
        double yaw_amount = 0.0;
        double pitch_amount = 0.0;
        double disp = 0.1;

        if(msg.command=="torso_up") pitch_amount = -disp;
        if(msg.command=="torso_down") pitch_amount = disp;
        if(msg.command=="torso_left") yaw_amount = disp;
        if(msg.command=="torso_right") yaw_amount = -disp;        
        
        q_desired[model.iDyn3_model.getDOFIndex("WaistSag")] = q_desired[model.iDyn3_model.getDOFIndex("WaistSag")] + pitch_amount;
        q_desired[model.iDyn3_model.getDOFIndex("WaistYaw")] = q_desired[model.iDyn3_model.getDOFIndex("WaistYaw")] + yaw_amount;

        if(msg.command=="torso_pitch")
        {
            q_desired[model.iDyn3_model.getDOFIndex("WaistSag")] = DEG2RAD* msg.deg_amount;
            exec_time=3.0;
        }
        if(msg.command=="torso_yaw")
        {
            q_desired[model.iDyn3_model.getDOFIndex("WaistYaw")] = DEG2RAD* msg.deg_amount;
            exec_time=3.0;
        }

        return true;
    }

	if(msg.command=="switch")
	{
		if(msg.frame=="l_sole")
		{
			current_chain = "wb_left";
		}
		if(msg.frame=="r_sole")
		{
			current_chain = "wb_right";
		}
		IK.update_model(current_chain,input);
		done=true;
		return true;
	}
	
	if(msg.command=="com_on_left")
	{
		current_chain = "wb_left";
	}
	if(msg.command=="com_on_right")
	{
		current_chain = "wb_right";
	}
	
	if(!initialized.at(current_chain))
	{
		IK.initialize(current_chain,input);
		initialized.at(current_chain)=true;
	}
	
	IK.update_model(current_chain,input);
	IK.get_current_wb_poses(current_chain,initial_poses);
	
	if( std::find(special_commands.begin(), special_commands.end(), msg.command)!=special_commands.end() )
	{
		double offset_x=0;
		double offset_y=0;
		double offset_z=0;
		
		if(msg.command=="hands_up") offset_z=0.1;
		else if(msg.command=="hands_down") offset_z=-0.1;
		else if(msg.command=="hands_forward") offset_x=0.1;
		else if(msg.command=="hands_backward") offset_x=-0.1;
		else if(msg.command=="hands_wide") offset_y=-0.1;
		else if(msg.command=="hands_tight") offset_y=0.1;
		
		msg.desired_poses["LSoftHand"] = initial_poses.at("LSoftHand") * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(offset_z,-offset_y,-offset_x)); // :3
		msg.desired_poses["RSoftHand"] = initial_poses.at("RSoftHand") * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(offset_z,offset_y,-offset_x));
		msg.desired_poses["l_sole"] = initial_poses.at("l_sole");
		msg.desired_poses["r_sole"] = initial_poses.at("r_sole");
		
		double com_offset_x=0;
		double com_offset_y=0;
		double com_offset_z=0;
		
		if(msg.command=="com_up") com_offset_z=0.1;
		else if(msg.command=="com_down") com_offset_z=-0.1;
		
		msg.desired_poses["COM"] = initial_poses.at("COM") * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(com_offset_x,com_offset_y,com_offset_z));
		
		if(msg.command=="com_on_left")
		{
			msg.desired_poses.at("COM").p.x(0.0);
			msg.desired_poses.at("COM").p.y(-0.025);
			msg.desired_poses.at("LSoftHand").p.y(0.35);
			msg.desired_poses.at("RSoftHand").p.y(-0.35);
		}
		else if(msg.command=="com_on_right")
		{
			msg.desired_poses.at("COM").p.x(0.0);
			msg.desired_poses.at("COM").p.y(0.025);
			msg.desired_poses.at("LSoftHand").p.y(0.35);
			msg.desired_poses.at("RSoftHand").p.y(-0.35);
		}
	}
	
	IK.set_desired_wb_poses_as_current(current_chain);
	
	exec_time = msg.duration;
	
	reset_traj_types();
	
	if(msg.command=="poses") std::cout<<"Request: "<<std::endl;
	next_poses.clear();

	IK.get_current_wb_poses(current_chain,next_poses);

	for(auto pose:msg.desired_poses)
	{
		if(msg.command=="poses")
		{
			std::cout<<" - "<<pose.first<<std::endl;
			if(pose.first!="COM") traj_types[pose.first] = msg.traj_type; //com is always linear
		}

		next_poses.at(pose.first) = pose.second;
	}

	IK.set_desired_wb_poses(current_chain,next_poses);

	double cart_error = IK.cartToJnt(current_chain,input,q_desired,0.005);
	
	if(cart_error==-1)
	{
		std::cout<<" !! ERROR in IK !! ( "<<current_chain<<" ) -> I won't move."<<std::endl;
		IK.update_model(current_chain,input); //to reset
		done=true;
		return false;
	}

	time=0;

	q_init = input;
	return true;
}

bool multicontact_thread::generate_touch_poses()
{
    done=false;
      
    if(!initialized.at(current_chain))
    {
        IK.initialize(current_chain,input);
        initialized.at(current_chain)=true;
    }
    
    if(msg.touch.at("l_sole")) // just to be sure
    {
        current_chain = "wb_right";
    }
    if(msg.touch.at("r_sole"))
    {
        current_chain = "wb_left";
    }

    IK.update_model(current_chain,input);
    IK.get_current_wb_poses(current_chain,initial_poses);

    msg.desired_poses.clear();
    for(auto t:msg.touch)
    {
//         msg.desired_poses[t.first] = KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(0,0,(t.second)?Z_OFFSET:0)) * initial_poses.at(t.first); // old solution, w.r.t. base

        if(t.second)
        {
            if(t.first=="l_sole") msg.desired_poses[t.first] = initial_poses.at(t.first) * KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(0,0,TOUCH_OFFSET));
            if(t.first=="r_sole") msg.desired_poses[t.first] = initial_poses.at(t.first) * KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(0,0,TOUCH_OFFSET));
            if(t.first=="LSoftHand") msg.desired_poses[t.first] = initial_poses.at(t.first) * KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(0,TOUCH_OFFSET,0));
            if(t.first=="RSoftHand") msg.desired_poses[t.first] = initial_poses.at(t.first) * KDL::Frame(KDL::Rotation::Identity(),KDL::Vector(0,-TOUCH_OFFSET,0));
        }

        traj_types.at(t.first) = 0;
    }
    
    IK.set_desired_wb_poses_as_current(current_chain);
    IK.set_desired_wb_poses(current_chain,msg.desired_poses);
    
    exec_time = msg.duration;

    std::cout<<"Touch: "<<std::endl;
    
    for(auto pose:msg.desired_poses)
    {
        if(msg.touch.at(pose.first)) std::cout<<" - "<<pose.first<<std::endl;
    }

    double cart_error = IK.cartToJnt(current_chain,input,q_desired,0.005);

	if(cart_error==-1)
	{
		std::cout<<" !! ERROR in IK !! ( "<<current_chain<<" ) -> I won't move."<<std::endl;
		IK.update_model(current_chain,input); //to reset
		done=true;
		return false;
	}

    time=0;

    q_init = input;
    return true;
}

void multicontact_thread::go_in_initial_position()
{
    q_init = input;
    going_to_initial_position = true;
	q_desired = home;
}

void multicontact_thread::reset_traj_types()
{
	for(auto& tt:traj_types) tt.second=0;
}

void multicontact_thread::setup_wb_ik() {
  
	chains.push_back("wb_left");
	chains.push_back("wb_right");
	
	base_frames["wb_left"] = "l_sole";
	base_frames["wb_right"] = "r_sole";
	
	for(auto frame:base_frames)
	{
		base_indeces[frame.first] = model.iDyn3_model.getLinkIndex(frame.second);
		initialized[frame.first] = false;
	}
	
	current_chain = "wb_left";
	
	ee_names.push_back("LSoftHand");
	ee_names.push_back("RSoftHand");
	ee_names.push_back("l_sole");
	ee_names.push_back("r_sole");
	
	for(auto name:ee_names)
	{
		ee_indeces.emplace(name,model.iDyn3_model.getLinkIndex(name));
		traj_types[name] = 0;
	}
	traj_types["COM"] = 0;
	
	
	available_commands.push_back("hands_up");
	available_commands.push_back("hands_down");
	available_commands.push_back("hands_forward");
	available_commands.push_back("hands_backward");
	available_commands.push_back("hands_wide");
	available_commands.push_back("hands_tight");
	available_commands.push_back("com_on_left");
	available_commands.push_back("com_on_right");
	available_commands.push_back("com_up");
	available_commands.push_back("com_down");
	
	for(auto cmd:available_commands) special_commands.push_back(cmd);
	
	available_commands.push_back("switch");
	available_commands.push_back("poses");
	available_commands.push_back("head_up");
	available_commands.push_back("head_down");
	available_commands.push_back("head_left");
	available_commands.push_back("head_right");
    available_commands.push_back("torso_up");
    available_commands.push_back("torso_down");
    available_commands.push_back("torso_left");
    available_commands.push_back("torso_right");
    available_commands.push_back("torso_pitch");
    available_commands.push_back("torso_yaw");


	square_duration = duration * 3.0;
}

void multicontact_thread::broadcast_com_tf()
{
	if(initialized.at(current_chain))
	{
		tf::Transform transform;
		IK.get_current_wb_poses(current_chain,initial_poses);
		tf::transformKDLToTF(initial_poses.at("COM"),transform);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_frames.at(current_chain), "COM"));
		ros::spinOnce();
	}
}