#include "soccer.hpp"

void stop(float posiciones2[40][6], const int rows) {
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();

  //Detenerse
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(posiciones2[rows-1][0]);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(posiciones2[rows-1][1]);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(posiciones2[rows-1][2] + rest_inc);
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(posiciones2[rows-1][3]);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(posiciones2[rows-1][4]);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(posiciones2[rows-1][5] - rest_inc);
  write_joint_pub.publish(write_msg);
  ros::Duration(0.1).sleep();
}

void walkForward(float posiciones2[40][6], const int rows) {
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();

  ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(0.7520);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(1.5317);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(-0.8143-rest_inc);
  write_joint_pub.publish(write_msg);
  
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(0.52356);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(1.14199);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(-0.618435-rest_inc);
  write_msg.name.push_back("r_ank_pitch");	//Pie derecho se acomoda para que centro de masa quede en medio de ambos pies
  write_msg.position.push_back(-0.618435);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.14199);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(0.52356+rest_inc);
  write_joint_pub.publish(write_msg);
  
  //Pie derecho
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(-0.7520);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.5317);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(0.8143+rest_inc);
  write_joint_pub.publish(write_msg);
  
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");	
  write_msg.position.push_back(-0.52356);		//-0.5486  <- valor matlab
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.14199);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(0.618435+rest_inc);
  write_msg.name.push_back("l_ank_pitch");	//Pie izquierdo se acomoda para que centro de masa quede en medio de ambos pies
  write_msg.position.push_back(0.618435);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(1.14199);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(-0.52356-rest_inc);
  write_joint_pub.publish(write_msg);
}

void turnRight(float posiciones2[40][6], const int rows) {
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();
  
  //Pie derecho
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(-0.7520);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.5317);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(0.8143+rest_inc);
  write_joint_pub.publish(write_msg);
  
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");	
  write_msg.position.push_back(-0.52356);		//-0.5486  <- valor matlab
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.14199);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(0.618435+rest_inc);
  write_msg.name.push_back("l_ank_pitch");	//Pie izquierdo se acomoda para que centro de masa quede en medio de ambos pies
  write_msg.position.push_back(0.618435);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(1.14199);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(-0.52356-rest_inc);
  write_joint_pub.publish(write_msg);

 ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(0.7520);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(1.5317);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(-0.8143-rest_inc);
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(posiciones2[rows-1][0]);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(posiciones2[rows-1][1]);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(posiciones2[rows-1][2]+rest_inc);
  write_joint_pub.publish(write_msg);
  
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(posiciones2[rows-1][3]);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(posiciones2[rows-1][4]);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(posiciones2[rows-1][5]-rest_inc);
  write_joint_pub.publish(write_msg);
}

void turnLeft(float posiciones2[40][6], const int rows) {
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();

  //Pie izquierdo
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(0.7520);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(1.5317);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(-0.8143-rest_inc);
  write_joint_pub.publish(write_msg);
  
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_ank_pitch");	
  write_msg.position.push_back(0.52356);		//-0.5486  <- valor matlab
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(1.14199);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(-0.618435-rest_inc);
  write_msg.name.push_back("r_ank_pitch");	//Pie izquierdo se acomoda para que centro de masa quede en medio de ambos pies
  write_msg.position.push_back(-0.618435);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.14199);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(0.52356+rest_inc);
  write_joint_pub.publish(write_msg);

//Pie derecho
 ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(-0.7520);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.5317);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(0.8143+rest_inc);
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(posiciones2[rows-1][3]);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(posiciones2[rows-1][4]);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(posiciones2[rows-1][5]-rest_inc);
  write_joint_pub.publish(write_msg);
  
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(posiciones2[rows-1][0]);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(posiciones2[rows-1][1]);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(posiciones2[rows-1][2]+rest_inc);
  write_joint_pub.publish(write_msg);
}

void walkLeft(float posiciones2[40][6], const int rows) {
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();

  //Levantar pie izquierdo 
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(0.8);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(1.8);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(-0.8143 - rest_inc);

  write_msg.name.push_back("l_hip_roll");
  write_msg.position.push_back(-0.0873*2);
  write_msg.name.push_back("r_hip_roll");
  write_msg.position.push_back(0.0873*2);
  write_msg.name.push_back("l_ank_roll");
  write_msg.position.push_back(-0.0873*2);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0.0873*2);
  write_joint_pub.publish(write_msg);

  //Bajar pie izquierdo
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(posiciones2[rows-1][3]);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(posiciones2[rows-1][4]);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(posiciones2[rows-1][5] - rest_inc);
  write_joint_pub.publish(write_msg);
  
  //Levantar pie derecho
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(-0.8);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.8);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(+0.8143 + rest_inc);
  write_msg.name.push_back("l_hip_roll");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("l_ank_roll");
  write_msg.position.push_back(-0.0873);
  write_joint_pub.publish(write_msg);
  
  //Bajar pie derecho
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(posiciones2[rows-1][0]);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(posiciones2[rows-1][1]);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(posiciones2[rows-1][2] + rest_inc);
  write_msg.name.push_back("r_hip_roll");
  write_msg.position.push_back(0.0873);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0.0873);
  write_joint_pub.publish(write_msg);
}

void walkRight(float posiciones2[40][6], const int rows) {
  sensor_msgs::JointState write_msg;
  write_msg.header.stamp = ros::Time::now();

  //Levantar pie izquierdo 
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(-0.8);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(-1.8);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(+0.8143 + rest_inc);

  write_msg.name.push_back("l_hip_roll");
  write_msg.position.push_back(-0.0873*2);
  write_msg.name.push_back("r_hip_roll");
  write_msg.position.push_back(0.0873*2);
  write_msg.name.push_back("l_ank_roll");
  write_msg.position.push_back(-0.0873*2);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0.0873*2);
  write_joint_pub.publish(write_msg);

  //Bajar pie izquierdo
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("r_ank_pitch");
  write_msg.position.push_back(posiciones2[rows-1][0]);
  write_msg.name.push_back("r_knee");
  write_msg.position.push_back(posiciones2[rows-1][1]);
  write_msg.name.push_back("r_hip_pitch");
  write_msg.position.push_back(posiciones2[rows-1][2]+rest_inc);
  write_joint_pub.publish(write_msg);
  
  //Levantar pie derecho
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(0.8);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(1.8);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(-0.8143-rest_inc);
  write_msg.name.push_back("r_hip_roll");
  write_msg.position.push_back(0.0873);
  write_msg.name.push_back("r_ank_roll");
  write_msg.position.push_back(0.0873);
  write_joint_pub.publish(write_msg);
  
  //Bajar pie derecho
  ros::Duration(0.1).sleep();
  write_msg.name.push_back("l_ank_pitch");
  write_msg.position.push_back(posiciones2[rows-1][3]);
  write_msg.name.push_back("l_knee");
  write_msg.position.push_back(posiciones2[rows-1][4]);
  write_msg.name.push_back("l_hip_pitch");
  write_msg.position.push_back(posiciones2[rows-1][5]-rest_inc);
  write_msg.name.push_back("l_hip_roll");
  write_msg.position.push_back(-0.0873);
  write_msg.name.push_back("l_ank_roll");
  write_msg.position.push_back(-0.0873);
  write_joint_pub.publish(write_msg);
}
