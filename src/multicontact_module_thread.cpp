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

using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using namespace walkman;

multicontact_thread::multicontact_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr< paramHelp::ParamHelperServer > ph):
control_thread( module_prefix, rf, ph ), recv_interface("multicontact_interface"),
// for FT_sensors filtering
WINDOW_size(5),
SENSORS_WINDOW(24,WINDOW_size),
SENSORS_SUM(24, 0.0), 
SENSORS_FILTERED(24, 0.0),
// for contact force vector calculation
map_l_fcToSens_PINV(12,6) ,
map_r_fcToSens_PINV(12,6) ,
map_l_hand_fcToSens_PINV(12,6) ,
map_r_hand_fcToSens_PINV(12,6)
//
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
    wb_cmd.add_command("idle");
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
    
    // TODO sensor force biasing

    return true;
}


void multicontact_thread::run()
{   
    sense();

    // get the command
    if(recv_interface.getCommand(msg,recv_num))
    {
      if(wb_cmd.parse_cmd(msg)) {
	
      } else {
	std::cout << "Something bad happened" << std::endl;
      }
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
    
    // TODO Filtering
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
  
  if(wb_cmd.going_to_initial_position) {
    wb_cmd.compute_q(time,output);
  }
    
  // filtro dq
  output = q_init;
}

void multicontact_thread::move()
{
    robot.move(output);
}