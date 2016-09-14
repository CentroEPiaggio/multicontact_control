#include <wb_interface/wb_interface.h>

wb_interface::wb_interface() {
std::cout << "wb_interface initialized" << std::endl;
}

wb_interface::~wb_interface() {
std::cout << "wb_interface deinitialized" << std::endl;
}

void wb_interface::get_q(double time, yarp::sig::Vector q) {
  
}



bool wb_interface::parse_cmd(multicontact_msg msg) {
  if (std::find(available_cmds.begin(), available_cmds.end(), msg.command) != available_cmds.end())
      {
	print_info(msg.command);
	if(msg.command=="reset")
        {
//             go_in_initial_position();
            time=0;
        }
        else
        {
// 		if(!generate_poses_from_cmd(msg.command))
// 		{
// 			std::cout<<"Received malformed command, abort"<<std::endl;
// 			return;
// 		}
            time=0;
        }
      }
      else
      {
	print_error(msg.command);
	return false;
      }
      return true;
}

void wb_interface::add_command(std::string cmd) {
  available_cmds.push_back(cmd);
}

void wb_interface::print_info(std::string str) { //TODO green
  std::cout << "WB_INTERFACE " << str << std::endl;
}

void wb_interface::print_error(std::string str) { //TODO red
  std::cout << "WB_INTERFACE " << str << std::endl;
}

void wb_interface::print_warning(std::string str) {// TODO yellow
  std::cout << "WB_INTERFACE " << str << std::endl;
}