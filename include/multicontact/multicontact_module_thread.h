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

#ifndef MULTICONTACT_THREAD_H_
#define MULTICONTACT_THREAD_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <idynutils/yarp_single_chain_interface.h>
#include <idynutils/comanutils.h>

#include <GYM/control_thread.hpp>
#include <GYM/yarp_command_interface.hpp>
#include <drc_shared/yarp_msgs/multicontact_msg.h>

#include <multicontact/multicontact.h>
#include <trajectory_generator/trajectory_generator.h>

#include <vector>

#include <wb_interface/wb_interface.h>

#include <locoman/utils/declarations.h>


/**
 * @brief multicontact control thread
 * 
 **/
namespace walkman
{
    class multicontact_thread : public control_thread
    {
    private:
        void control_law();

        yarp::sig::Vector input;
        yarp::sig::Vector output;
        yarp::sig::Vector home;
        yarp::sig::Vector q_init;
	yarp::sig::Vector ft_l_ankle;
	yarp::sig::Vector ft_r_ankle;
	yarp::sig::Vector ft_l_wrist;
	yarp::sig::Vector ft_r_wrist;
	
	//
	unsigned int size_q;
   
     
    yarp::sig::Vector q_sense;
    yarp::sig::Vector q_current;
    yarp::sig::Vector q_offset;   // q_current = q_sense + q_offset
    
    yarp::sig::Vector q_des;
    
	// Variables for FT_sensor filtering
	yarp::sig::Vector Sensor_Collection;
        
	yarp::sig::Matrix SENSORS_WINDOW ;
	yarp::sig::Vector SENSORS_SUM ;
	yarp::sig::Vector SENSORS_FILTERED ;
    
	int count_sensor;
	int WINDOW_size;
	//
        multicontact_msg msg;
        walkman::yarp_custom_command_interface<multicontact_msg> recv_interface;
        int recv_num=0;
	

        double time = 0;
        double duration = 3.0;
	
// 	wb_interface wb_cmd; // interface for wholebody IK control
	
	// contact force vector section begin
	yarp::sig::Vector fc_l_c_to_world; 
	yarp::sig::Vector fc_r_c_to_world;
	yarp::sig::Vector fc_feet_to_world;
	
	yarp::sig::Vector fc_l_c_hand_to_world;
	yarp::sig::Vector fc_r_c_hand_to_world;
	yarp::sig::Vector fc_hand_to_world;
	
	yarp::sig::Matrix map_l_fcToSens_PINV;
	yarp::sig::Matrix map_r_fcToSens_PINV;
	yarp::sig::Matrix map_l_hand_fcToSens_PINV;
	yarp::sig::Matrix map_r_hand_fcToSens_PINV;
	
	yarp::sig::Vector fc_offset_left;
	yarp::sig::Vector fc_offset_right;
    
	yarp::sig::Vector fc_offset_left_hand;
	yarp::sig::Vector fc_offset_right_hand;
	
	yarp::sig::Vector fc_current_left;
	yarp::sig::Vector fc_current_right;
	yarp::sig::Vector fc_current_left_hand;
	yarp::sig::Vector fc_current_right_hand;

	yarp::sig::Vector FC_to_world;

	bool flag_robot = 1 ;
	bool flag_simulator = 1-flag_robot ;
	
	           
    unsigned int waist_index ;
    unsigned int l_ankle_index ;
    unsigned int l_c1_index ;
    unsigned int l_c2_index ;
    unsigned int l_c3_index ;
    unsigned int l_c4_index ;
    unsigned int r_ankle_index ;
    unsigned int r_c1_index ;
    unsigned int r_c2_index ;
    unsigned int r_c3_index ;
    unsigned int r_c4_index ;
    unsigned int l_hand_index ;
    unsigned int r_hand_index ;
    unsigned int l_wrist_index ;
    unsigned int l_hand_c1_index ;
    unsigned int l_hand_c2_index ;
    unsigned int l_hand_c3_index ;
    unsigned int l_hand_c4_index ;
    unsigned int r_wrist_index ;
    unsigned int r_hand_c1_index ;
    unsigned int r_hand_c2_index ;
    unsigned int r_hand_c3_index ;
    unsigned int r_hand_c4_index ;
    
    yarp::sig::Vector fc_sense_left;
    yarp::sig::Vector fc_sense_right;
    yarp::sig::Vector fc_sense_left_hand;
    yarp::sig::Vector fc_sense_right_hand;
  
	void contact_force_vector_computation();
	// contact force vector section end
	
	void read_offset_q();
    public:
        /**
        * @brief constructor
        * 
        * @param module_prefix the prefix of the module
        * @param rf resource finderce
        * @param ph param helper
        */
        multicontact_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr<paramHelp::ParamHelperServer> ph );

        /**
        * @brief multicontact control thread initialization
        * 
        * @return true on succes, false otherwise
        */
        virtual bool custom_init();

        /**
        * @brief multicontact control thread main loop
        * 
        */
        virtual void run();

        /**
            * @brief sense function
            */
        void sense();

        /**
            * @brief move function
            */
        void move();
	
	/**
            * @brief send sensor data to locoman service 2
            */
	void send_to_service2();
    };
}

#endif // MULTICONTACT_THREAD_H_
