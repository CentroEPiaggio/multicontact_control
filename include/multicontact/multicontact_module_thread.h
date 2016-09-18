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

#include <multicontact/multicontact.h>
#include <trajectory_generator/trajectory_generator.h>

#include <vector>

#include <locoman/utils/declarations.h>

#include <wholebody_ik/wholebody_ik.h>
#include <drc_shared/yarp_msgs/multicontact_msg.h>
#include <trajectory_generator/trajectory_generator.h>
#include <utils/utils.h>

#include <drc_shared/data_logger.hpp>

/**
 * @brief multicontact control thread
 * 
 **/
namespace walkman
{
    class multicontact_thread : public control_thread
    {
    private:

    //------------------------------------------      
    // Yarp Ports ------------
    // Sending Ports
    yarp::os::BufferedPort<yarp::sig::Vector> sending_q    ;
    yarp::os::BufferedPort<yarp::sig::Vector> sending_fc   ;

    yarp::os::BufferedPort<yarp::sig::Matrix> receiving_Big_Rf;
    yarp::sig::Matrix *Big_Rf_received ;
    bool receiving_Big_Rf_initted ;
    yarp::sig::Matrix Big_Rf_data;  
    int run_counter ;
    
    // End of the Yarp Ports -------------------
    //------------------------------------------
      
    utils utils_1;
    walkman::yarp_custom_command_interface<multicontact_msg> recv_interface;
    wholebody_ik IK;
    unsigned int size_q;
   
     
    yarp::sig::Vector q_sense;
    yarp::sig::Vector q_current;
    yarp::sig::Vector q_offset;   // q_current = q_sense + q_offset
    
    yarp::sig::Vector q_des;
    
    yarp::sig::Vector d_q_move ;
    
    int WINDOW_size;

		
	yarp::sig::Matrix SENSORS_WINDOW ;
	yarp::sig::Vector SENSORS_SUM ;
	yarp::sig::Vector SENSORS_FILTERED ;
	yarp::sig::Vector SENSORS_AT_COMMAND ;
	
	
	yarp::sig::Vector ft_l_ankle;
	yarp::sig::Vector ft_r_ankle;
	yarp::sig::Vector ft_l_wrist;
	yarp::sig::Vector ft_r_wrist;
	
	
    
	// Variables for FT_sensor filtering
	yarp::sig::Vector Sensor_Collection;
        

    
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

	    yarp::sig::Vector fc_sense_left;
    yarp::sig::Vector fc_sense_right;
    yarp::sig::Vector fc_sense_left_hand;
    yarp::sig::Vector fc_sense_right_hand;

    yarp::sig::Vector FC_DES_NO_RIGHT_FOOT;
    yarp::sig::Vector FC_DES_NO_LEFT_FOOT;
    yarp::sig::Vector FC_DES_feet  ;
    yarp::sig::Vector FC_DES_hands ;
    yarp::sig::Vector d_fc_total_des ;
    
    yarp::sig::Matrix Big_Rf_new ; 

	bool flag_robot = 1 ;
	bool flag_simulator = 1-flag_robot ;
	
	           
	int count_sensor;
	//
        multicontact_msg msg;
        int recv_num=0;
	
	
	void control_law();

        yarp::sig::Vector input;
        yarp::sig::Vector output;
        yarp::sig::Vector home;
        yarp::sig::Vector q_init;
	
    int waist_index ;
    int l_ankle_index ;
    int l_c1_index ;
    int l_c2_index ;
    int l_c3_index ;
    int l_c4_index ;
    int r_ankle_index ;
    int r_c1_index ;
    int r_c2_index ;
    int r_c3_index ;
    int r_c4_index ;
    int l_hand_index ;
    int r_hand_index ;
    int l_wrist_index ;
    int l_hand_c1_index ;
    int l_hand_c2_index ;
    int l_hand_c3_index ;
    int l_hand_c4_index ;
    int r_wrist_index ;
    int r_hand_c1_index ;
    int r_hand_c2_index ;
    int r_hand_c3_index ;
    int r_hand_c4_index ;
    

  
	void contact_force_vector_computation();
	// contact force vector section end
	
	void read_offset_q();
	
	// from wb_ik thread module
	//
	void go_in_initial_position();
        bool going_to_initial_position=false;

        std::vector<std::string> available_commands;
		std::vector<std::string> special_commands;
        bool generate_poses_from_cmd();

    bool generate_touch_poses();
	
	double time = 0;
        double duration = 5.0;
	
	double square_duration;
	double exec_time;

	std::vector<std::string> chains;
	std::map<std::string,std::string> base_frames;
	std::map<std::string,int> base_indeces;
	std::string current_chain;

	std::vector<std::string> ee_names;
	std::map<std::string,int> ee_indeces;
	std::map<std::string,KDL::Frame> initial_poses;
	std::map<std::string,KDL::Frame> next_poses;
	std::map<std::string,trajectory_generator> traj_gens;
	std::map<std::string,int> traj_types;
	std::map<std::string,bool> initialized;
	bool done = false;

	void reset_traj_types();
	
	std::map<std::string,bool> state_map;
	void set_state(std::string key);
	void set_idle_state();

	bool is_state_active(std::string key);
	
	const double DELTA_F_MAX = 10.0; // threshold on force
	const double Z_OFFSET = -0.1; // z displacement for touching control
	std::map<std::string,KDL::Frame> touch_poses;
        
	double mg = 1200 ;
	double feet_part  = 2.0/3.0 ;
	double hands_part = 1.0-feet_part ;
	double mg_foot  = feet_part*mg ;
	double mg_hands = hands_part*mg ;
	double regu_filter = 1E9 ; 
	//
	walkman::log_utils::data_logger log_sense;
    walkman::log_utils::data_logger log_1;
    walkman::log_utils::data_logger log_2;
	walkman::log_utils::data_logger log_3;
    walkman::log_utils::data_logger log_4;

	tf::TransformBroadcaster br;
	void broadcast_com_tf();

    yarp::sig::Vector delta;
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
	
	/**
	 * @brief read dq from locoman service 2
	 */
	void read_from_service2(); 
	
	/**
	 * @brief control law purely based on wholebody inverse kinematics
	 */
	void control_law_ik();
	
	/**
	 * @brief setup attributes for wholebody_ik control
	 */
	void setup_wb_ik();
    };
}

#endif // MULTICONTACT_THREAD_H_
