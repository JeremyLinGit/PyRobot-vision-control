#include "ros/ros.h"
#include <std_msgs/String.h>
#include "navigation_controller/command.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <fstream>
using namespace std;

bool i_am_angry=false;

class CommandPubSrvGui {
public:
	CommandPubSrvGui();
	~CommandPubSrvGui();

private:
	ros::NodeHandle nh_;
	ros::Subscriber place_cmd_;
	ros::ServiceClient client_;
	ros::Publisher reach_pub_;


	ros::Publisher flag_switch;
	std_msgs::Bool cmd_switch;


	void get_place(const std_msgs::String::ConstPtr& place);
};

CommandPubSrvGui::CommandPubSrvGui()
{
	place_cmd_ = nh_.subscribe<std_msgs::String>("place", 1000, boost::bind(&CommandPubSrvGui::get_place, this, _1));
	client_ = nh_.serviceClient<navigation_controller::command>("pos_cmd");

	flag_switch = nh_.advertise<std_msgs::Bool>("yourturn", 1);
	//cmd_switch.data
}

CommandPubSrvGui::~CommandPubSrvGui()
{
	
}


void CommandPubSrvGui::get_place(const std_msgs::String::ConstPtr& place)
{
	int num = 1;
	int flag=0;
	int wait=0;
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;

	navigation_controller::command srv;

	std::string path = "/home/locobot/low_cost_ws_my/src/pyrobot/robots/LoCoBot/navigation_controller/cmd_txt/final.txt";
	std::fstream myfile;
	myfile.open(path.c_str());

	while (myfile >>num>>x>>y>>theta && ros::ok()) {
  ////////////////////////////////TODO/////////////////////////////// 
 
 
  ////////////////////////////////TODO///////////////////////////////
		ROS_INFO_STREAM("x = " << x << " y = " << y << " theta = " << theta);
		srv.request.type = num;
		srv.request.x = x;
		srv.request.y = y;
		srv.request.theta = theta;

		if(client_.call(srv)) {
			ROS_INFO("call service success");
			srv.request.type = 0;
			while(srv.response.run_completed == false) {
				usleep(100000);
				ROS_INFO("hello servie");
				client_.call(srv);
			}
			sleep(wait);
		}
		else {
			ROS_INFO("call service fail");
		}
	}
	cmd_switch.data = true;
	for(int i=0;i<1;i++){
		if(i_am_angry==false){
			i_am_angry=true;
			flag_switch.publish(cmd_switch);
			//ROS_ERROR("-----------------------------------FUCK");
			ros::Duration(1.0).sleep();
			//i_am_angry=true;
			printf("I am angry\n\n\n\n");
			return;
		}
	}
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "command_pub_srv_gui");
	CommandPubSrvGui commandpubsrvgui;

	ros::spin();	
	return 0;
}
