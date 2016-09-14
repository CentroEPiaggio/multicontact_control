#include <wb_interface/wb_interface.h>

using namespace wb_interface;
wb_interface() {
std::cout << "wb_interface initialized" << std::endl;
}
~wb_interface() {
std::cout << "wb_interface deinitialized" << std::endl;
}

void get_q(double time, yarp::sig::Vector q) {
  
}

void go_in_initial_position()
{
    q_init = input;
    going_to_initial_position = true;
}

bool generate_poses_from_cmd(std::string cmd)
{
    if( std::find(available_commands.begin(), available_commands.end(), cmd)==available_commands.end() )
    {
        std::cout<<" !! ERROR: command not available !!"<<std::endl;
        return false;
    }

	if(cmd=="com_on_left")
	{
		current_chain = "wb_left";
	}
	if(cmd=="com_on_right")
	{
		current_chain = "wb_right";
	}
	
	if(!initialized.at(current_chain))
	{
		IK.initialize(current_chain,input);
		initialized.at(current_chain)=true;
	}

	IK.get_current_wb_poses(current_chain,initial_poses);

	if(cmd!="poses")
	{
		double offset_x=0;
		double offset_y=0;
		double offset_z=0;

		if(cmd=="hands_up") offset_z=0.1;
		else if(cmd=="hands_down") offset_z=-0.1;
		else if(cmd=="hands_forward") offset_x=0.1;
		else if(cmd=="hands_backward") offset_x=-0.1;
		else if(cmd=="hands_wide") offset_y=-0.1;
		else if(cmd=="hands_tight") offset_y=0.1;

		msg.desired_poses["LSoftHand"] = initial_poses.at("LSoftHand") * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(offset_z,-offset_y,-offset_x)); // :3
		msg.desired_poses["RSoftHand"] = initial_poses.at("RSoftHand") * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(offset_z,offset_y,-offset_x));
		msg.desired_poses["l_sole"] = initial_poses.at("l_sole");
		msg.desired_poses["r_sole"] = initial_poses.at("r_sole");

		double com_offset_x=0;
		double com_offset_y=0;
		double com_offset_z=0;

		if(cmd=="com_up") com_offset_z=0.1;
		else if(cmd=="com_down") com_offset_z=-0.1;
		
		msg.desired_poses["COM"] = initial_poses.at("COM") * KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(com_offset_x,com_offset_y,com_offset_z));

		if(cmd=="com_on_left")
		{
			msg.desired_poses.at("COM").p.x(0.0);
			msg.desired_poses.at("COM").p.y(-0.04);
			msg.desired_poses.at("LSoftHand").p.y(0.35);
			msg.desired_poses.at("RSoftHand").p.y(-0.35);
		}
		else if(cmd=="com_on_right")
		{
			msg.desired_poses.at("COM").p.x(0.0);
			msg.desired_poses.at("COM").p.y(0.04);
			msg.desired_poses.at("LSoftHand").p.y(0.35);
			msg.desired_poses.at("RSoftHand").p.y(-0.35);
		}
	}

	IK.set_desired_wb_poses_as_current(current_chain);

	for(auto pose:msg.desired_poses) traj_gens.at(pose.first).line_initialize(duration,initial_poses.at(pose.first),pose.second);
	if(msg.desired_poses.count("COM")) traj_gens.at("COM").line_initialize(duration,initial_poses.at("COM"),msg.desired_poses.at("COM"));
	
	done=false;

    return true;
}

bool parse_cmd(multicontact_msg msg) {
  if (std::find(available_cmds.begin(), available_cmds.end(), msg.command) != available_cmds.end())
      {
	print_info("Message received: " << msg.command);
	if(msg.command=="reset")
        {
            go_in_initial_position();
            time=0;
        }
        else
        {
		if(!generate_poses_from_cmd(msg.command))
		{
			std::cout<<"Received malformed command, abort"<<std::endl;
			return;
		}
            time=0;
        }
      }
      else
      {
	print_error("Unknown command: " << msg.command);
	return false;
      }
      return true;
}

void add_command(std::string cmd) {
  available_cmds.push_back(cmd);
}

void print_info(std::string str) { //TODO green
  std::cout << "WB_INTERFACE " << str << std::endl;
}

void print_error(std::string str) { //TODO red
  std::cout << "WB_INTERFACE " << str << std::endl;
}

void print_warning(std::string str) {// TODO yellow
  std::cout << "WB_INTERFACE " << str << std::endl;
}