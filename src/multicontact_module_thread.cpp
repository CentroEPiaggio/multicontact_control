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
control_thread( module_prefix, rf, ph ), recv_interface("multicontact_interface")
{
    input.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    output.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    home.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    q_init.resize(model.iDyn3_model.getNrOfDOFs(),0.0);
    
    ft_readings = robot.senseftSensors(); // l_arm_ft, l_leg_ft, r_arm_ft, r_leg_ft
     
//     sample loop for printing ft readings
//     for(std::map<std::string,yarp::sig::Vector>::iterator it = ft_readings.begin(); it!=ft_readings.end();++it){
//         std::cout << it->first  << std::endl;
// 	std::cout << ft_readings[it->first].toString() << std::endl;
//     getchar();
//     }
    
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

    // command list
    available_cmds.push_back("idle");
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

    return true;
}


void multicontact_thread::run()
{   
    sense();

    // get the command
    if(recv_interface.getCommand(msg,recv_num))
    {
      if (std::find(available_cmds.begin(), available_cmds.end(), msg.command) != available_cmds.end())
      {
	std::cout<<"Command received: "<<msg.command<<std::endl;
      }
      else
      {
	std::cout<<"Unknown command: "<<msg.command<<std::endl;
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
    ft_readings = robot.senseftSensors(); // l_arm_ft, l_leg_ft, r_arm_ft, r_leg_ft
}

void multicontact_thread::control_law()
{
  
  
  // filtro dq
  output = q_init;
}

void multicontact_thread::move()
{
    robot.move(output);
}