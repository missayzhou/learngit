#ifndef LINE_H
#define LINE_H
#include <ros/ros.h>
#include "addObject.h"
#include "collisionCheck.h"
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <thread>
#include <chrono>
#include <fstream>
#include <unistd.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <serial/serial.h>  //ROS has built-in serial package
#include <opencv2/opencv.hpp>
#include<iostream>
#include "../include/grasp.h"

using namespace std;
using namespace cv;

extern geometry_msgs::PointStamped ball_lscene,ball_robot,ball_robot_sended;

union lz
{
    float temp;
    unsigned char moveit_arm_lz[4];
};

class x_arm_robot
{
private:
    moveit::planning_interface::MoveItErrorCode* success;
    moveit::planning_interface::MoveGroupInterface* group;
    moveit::planning_interface::MoveGroupInterface::Plan* my_plan;
    moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;
    std::vector<double> jointValue;
    geometry_msgs::Pose ballPose;
    ros::NodeHandle* nh;
    serial::Serial* moveit_arm;
    serial::Serial* ser_hand;
    IK_fast_solver::grasp* grasp_srv;
    ros::ServiceClient* grasp_client;
    geometry_msgs::Pose* target_pose;
    bool serialPortOpened;

    std::string usb_id;
    int time_delay;


    short int theta;//plus 1000 to reserve three digit after the dot.
    unsigned char* temp;//temp[1] is the higher 8 digits
    char c;
    unsigned char joint_angle_data[18],hand_pos_data[18],hand_cmd[6],calib_angle_data[18];

public:
    x_arm_robot(ros::NodeHandle* nh,IK_fast_solver::grasp* grasp_srv,ros::ServiceClient* grasp_client);
    ~x_arm_robot();
    void first_moveToTarget();
    void second_moveToTarget(tf::StampedTransform& joint6_robot);
    void three_moveToTarget();
    void send_pvt();
    void Ser_Arm_Initialize();
    void input_interface();
    void set_joint_angle(unsigned char* joint_angle_data,IK_fast_solver::grasp grasp_srv);
    void set_joint_angle_const(unsigned char* joint_angle_data,float angle0,float angle1,float angle2,float angle3,float angle4,float angle5,float angle6);
    void Ser_Hand_Initialize();
};

void pos_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
void transformPoint(const tf::TransformListener &listener);

#endif // LINE_H
