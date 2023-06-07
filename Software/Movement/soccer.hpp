/* Author: Pedro y Marlene */
#ifndef SOCCER_H
#define SOCCER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Eigen>

#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "op3_action_module_msgs/IsRunning.h"

void readyToDemo();
void setModule(const std::string& module_name);
void goInitPose();
void goAction(int page);
bool checkManagerRunning(std::string& manager_name);
void torqueOnAll();
bool isActionRunning();

void callbackMotions(const std_msgs::Int32MultiArray::ConstPtr& msg);
void callbackError(const geometry_msgs::Point& msg);
void callbackPosition(const geometry_msgs::Point& msg);
void callbackImu(const sensor_msgs::Imu::ConstPtr& msg);

void stop(float posiciones2[40][6], const int rows) ;
void walkForward(float posiciones2[40][6], const int rows);
void turnLeft(float posiciones2[40][6], const int rows);
void turnRight(float posiciones2[40][6], const int rows);
void walkLeft(float posiciones2[40][6], const int rows);
void walkRight(float posiciones2[40][6], const int rows);

const double FALL_FORWARD_LIMIT = 55;
const double FALL_BACK_LIMIT = -50;
double present_pitch_;
int page;
int state;

char action;

int ult_pos;
double rest_inc = 0.2181;
double rest_inc_giro = 0.2181;

double t_ref_ang;
double t_ref;
double act_val = 0;

float posiciones2[40][6];
const int rows = 40;
const int cols = 6;

enum ControlModule
{
  None = 0,
  DirectControlModule = 1,
  Framework = 2,
};

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = false;

ros::Publisher init_pose_pub;
ros::Publisher action_pose_pub;
ros::Publisher dxl_torque_pub;
ros::Publisher write_joint_pub;
ros::Subscriber imu_sub;
ros::Subscriber motions_sub;
ros::ServiceClient set_joint_module_client;
ros::ServiceClient is_running_client;

int control_module = None;
bool demo_ready = false;

#endif 
