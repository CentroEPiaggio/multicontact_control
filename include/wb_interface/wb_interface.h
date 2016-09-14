#ifndef __WB_INTERFACE_H
#define __WB_INTERFACE_H

#include <wholebody_ik/wholebody_ik.h>
#include <drc_shared/yarp_msgs/multicontact_msg.h>

class wb_interface {
private:
wholebody_ik IK;

std::vector<std::string> available_cmds;

double time=0;


public:
  bool going_to_initial_position = false;

  wb_interface();
  ~wb_interface();
  
  /**
   * @brief calculates output q
   */
  void get_q(double time, yarp::sig::Vector q);

  
  /**
   * @brief parse commands
   */
  bool parse_cmd(multicontact_msg msg);
  
  /**
   * @brief add feasible command string
   */
  void add_command(std::string cmd);
  
  // Utils functions
  void print_info(std::string str);

  void print_error(std::string str);

  void print_warning(std::string str);
  
};

#endif // __WB_INTERFACE_H
