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


using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using namespace walkman;

multicontact_thread::multicontact_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr< paramHelp::ParamHelperServer > ph):
	control_thread( module_prefix, rf, ph ), recv_interface("multicontact_interface"),
// size of q vectors
	size_q(locoman::utils::getNumberOfKinematicJoints(robot)),
	q_sense(size_q,0.0),
	q_current(size_q, 0.0),
	q_offset(size_q, 0.0),   // q_current = q_sense + q_offset
	q_des(size_q, 0.0),
// for FT_sensors filtering
	WINDOW_size(5),
	SENSORS_WINDOW(24,WINDOW_size),
	SENSORS_SUM(24, 0.0), 
	SENSORS_FILTERED(24, 0.0),
//
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
	fc_sense_right_hand(12,0.0)
  
{
    input.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    output.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    home.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    q_init.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    
    yarp::sig::Vector q_right_arm(7,0.0);
    yarp::sig::Vector q_left_arm(7,0.0);
    yarp::sig::Vector q_torso(3,0.0);
    yarp::sig::Vector q_right_leg(6,0.0);
    yarp::sig::Vector q_left_leg(7,0.0);
    yarp::sig::Vector q_head(2,0.0);

    q_head[0] = 0.0;
    q_head[1] = 0.0;

    q_right_arm[0]=  0.6;
    q_right_arm[1]= -0.2;
    q_right_arm[3]= -1.2;
    q_right_arm[5]= -0.6;

    q_left_arm[0]=  0.6;
    q_left_arm[1]=  0.2;
    q_left_arm[3]= -1.2;
    q_left_arm[5]= -0.6;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
 
    q_right_leg[2]= -0.3;
    q_right_leg[3]=  0.6;
    q_right_leg[4]= -0.3;
    
    q_left_leg[2]= -0.3;
    q_left_leg[3]=  0.6;
    q_left_leg[4]= -0.3;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,home);

    // populate command list
//     wb_cmd.add_command("idle");
}

bool multicontact_thread::custom_init()
{
    //  real time thread
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam ( pthread_self(), SCHED_FIFO, &thread_param );
    
    // setup
    model.iDyn3_model.setFloatingBaseLink(model.left_leg.end_effector_index);
    sense();
    output = input;
    robot.setPositionDirectMode();


    std::cout<<" - Initialized"<<std::endl;

	//------------------------------------------------------------------------------------------
	char vai_01 ;
	std::cout << " Put the Robot UP on the terrain and press a key !!! " << std::endl ;
	//std::cout << " waiting for a keyboard input !!! " << std::endl ;
	std::cin >> vai_01 ;
//  //-------------------
    
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
	int dim_fc_offset = 1000 ;  
 
	fc_offset_left.zero()  ;
	fc_offset_right.zero() ;
	fc_offset_left_hand.zero() ;
	fc_offset_right_hand.zero() ;
 
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
  
	// TODO
	// filling sensors window
  
}

void multicontact_thread::read_offset_q(){
	// offset on q
	// q_ Offset Evaluation Section
	int dim_offeset = 1000    ; 
	yarp::sig::Matrix offset_window(locoman::utils::getNumberOfKinematicJoints(robot), dim_offeset);
	q_current.zero();
	for(int k=0; k<dim_offeset ; k++ ){
		q_current += locoman::utils::sense_position_no_hands(robot); //if sense returns motorPosition       
		usleep(1*1000) ;  
	}   
	q_current = q_current/dim_offeset ;  
  
	// yarp::sig::Vector q_motor_init = q_des  ;
	q_offset = q_des - q_current ;
	// end of the Homing Section
  
	q_sense =  locoman::utils::senseMotorPosition(robot, flag_robot) ;
	q_current = q_sense + q_offset ; 
	std::cout << " final error offset =  " <<  norm(q_current - q_des) << std::endl;     

}

void multicontact_thread::send_to_service2() {
	// TODO
}

void multicontact_thread::run()
{   
    sense();

    send_to_service2();
    
    // get the command
    if(recv_interface.getCommand(msg,recv_num))
    {
//       if(wb_cmd.parse_cmd(msg)) {
// 	
//       } else {
// 	std::cout << "Something bad happened" << std::endl;
//       }
    }

    control_law();

    move();
}

void multicontact_thread::sense()
{
    // joint positions
    input = robot.sensePosition();
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
  
	std::cout << "Sensor_Collection = "  << Sensor_Collection.toString() << std::endl ;  

	count_sensor = count_sensor% WINDOW_size ;
	SENSORS_WINDOW.setCol( count_sensor , Sensor_Collection ) ;
	SENSORS_SUM = SENSORS_SUM + Sensor_Collection -1.0 * SENSORS_WINDOW.getCol((count_sensor+ 1)%WINDOW_size) ; 
	SENSORS_FILTERED = SENSORS_SUM / (WINDOW_size-1.0) ;
	count_sensor += 1 ;
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
  
//   if(wb_cmd.going_to_initial_position) {
//     wb_cmd.compute_q(time,output);
//   } else {
//     wb_cmd.get_q(output);
//   }
    
	// filtro dq
	output = q_init;
}

void multicontact_thread::move()
{
    robot.move(output);
}