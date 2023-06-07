#include "soccer.hpp"
#include "actions.hpp"

//node main
int main(int argc, char **argv) {
  
  //init ros
  ros::init(argc, argv, "soccer");
  ros::NodeHandle nh(ros::this_node::getName());

  std::ifstream myfile ("/home/robotis/nayarit_ws/src/op3_leo/data/Pararse.txt");
  if (myfile.is_open()) {
	std::cout << "El archivo se abrio" << std::endl;
	
		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < cols; j++) {
				myfile >> posiciones2[i][j];
			}
		}
		myfile.close();
  } else {
  	std::cout << "El archivo no abrio" << std::endl;
  }

  //subscribers
  ros::Subscriber imu_sub = nh.subscribe("/robotis/open_cr/imu", 1, callbackImu);
  ros::Subscriber motions_sub = nh.subscribe("/motions_left", 10, callbackMotions);

  //publishers
  init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  action_pose_pub = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
  dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  write_joint_pub = nh.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 0);
    
  //services
  set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");
  is_running_client = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");
  
  ros::start();

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  //wait for starting of op3_manager
  std::string manager_name = "/op3_manager";
  while (ros::ok()) {
    ros::Duration(1.0).sleep();

    if (checkManagerRunning(manager_name) == true) {
      break;
      ROS_INFO_COND(DEBUG_PRINT, "Succeed to connect");
    }
    ROS_WARN("Waiting for op3 manager");
  }

  readyToDemo();

  //node loop
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();
  
  //standup
  ros::Duration(1).sleep();
  ros::Rate loop_rate_pararse(60);
  
  for (int i=0; i<rows; i++) {
    write_msg.name.push_back("r_ank_pitch");
    write_msg.position.push_back(posiciones2[i][0]);
    write_msg.name.push_back("r_knee");
    write_msg.position.push_back(posiciones2[i][1]);
    write_msg.name.push_back("r_hip_pitch");
    write_msg.position.push_back(posiciones2[i][2]+rest_inc);
    write_msg.name.push_back("l_ank_pitch");
    write_msg.position.push_back(posiciones2[i][3]);
    write_msg.name.push_back("l_knee");
    write_msg.position.push_back(posiciones2[i][4]);
    write_msg.name.push_back("l_hip_pitch");
    write_msg.position.push_back(posiciones2[i][5]-rest_inc);
    write_joint_pub.publish(write_msg);
        
    loop_rate_pararse.sleep();
  }
    
  //feet arrange
  ros::Duration(1).sleep();
  write_msg.name.push_back("l_hip_yaw");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_hip_yaw");
  write_msg.position.push_back(0.0873);
  write_joint_pub.publish(write_msg);
        
  ros::Duration(1).sleep();
  write_msg.name.push_back("l_hip_roll");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_hip_roll");
  write_msg.position.push_back(0.0873);
  write_msg.name.push_back("l_ank_roll");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0.0873);
  write_joint_pub.publish(write_msg);

  write_msg.name.push_back("head_pan");
  write_msg.position.push_back(0);
  write_msg.name.push_back("head_tilt");
  write_msg.position.push_back(0.349066);
  write_joint_pub.publish(write_msg);

  return 0;
}

void readyToDemo() {
  ROS_INFO("Start read-write demo");
  torqueOnAll();
  ROS_INFO("Torque on all joints");

  //send message for going init posture
  goInitPose();
  ROS_INFO("Init pose");

  //wait while ROBOTIS-OP3 goes to the init posture
  ros::Duration(4.0).sleep();

  setModule("none");
}

void goInitPose() {
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";
  init_pose_pub.publish(init_msg);
}

void goAction(int page) {
  setModule("action_module");
  ROS_INFO("Action pose");

  std_msgs::Int32 action_msg;
  action_msg.data = page;
  action_pose_pub.publish(action_msg);
}

bool checkManagerRunning(std::string& manager_name) {
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);

  for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
  {
    if (node_list[node_list_idx] == manager_name)
      return true;
  }
  ROS_ERROR("Can't find op3_manager");
  return false;
}

void setModule(const std::string& module_name) {
  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = module_name;

  if (set_joint_module_client.call(set_module_srv) == false)
  {
    ROS_ERROR("Failed to set module");
    return;
  }
  return ;
}

void torqueOnAll() {
  std_msgs::String check_msg;
  check_msg.data = "check";
  dxl_torque_pub.publish(check_msg);
}