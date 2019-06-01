#ifndef ZHOU_X_ARM_H
#define ZHOU_X_ARM_H
#include "../include/IK_fast_solver.h"

#include "addObject.h"
#include "collisionCheck.h"
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <ros/ros.h>

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>

using namespace std;
class lz_x_arm{
public:
    lz_x_arm();
    ~lz_x_arm();
    int armReset(unsigned char head=0x55,unsigned char tail=0x04);
    int onePoint();
    int twoPoint(IK_fast_solver::grasp grasp_srv,ros::ServiceClient grasp_client );
    int threepoint(IK_fast_solver::grasp grasp_srv,ros::ServiceClient grasp_client);
    void drink_water();
    void ik_fast_pose();
    void moveit_grasp();
    void plan();
    void lz_x_armExecute();

    void jointToByteArray();
    
    void addObjector();
    void setTargetPose();
    void getLoadPlan();
     int graspedObjectType=0; // 0:ball, 1:bottle
     
     tf::TransformListener* tf_listener;
private:
    moveit::planning_interface::MoveItErrorCode success;

    moveit::planning_interface::MoveGroupInterface* group;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    geometry_msgs::Pose ballPose;
 
    
    
    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose;
    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    geometry_msgs::PointStamped ball_lscene,ball_robot,ball_robot_sended;

    tf2::Quaternion myOritation;
   
    

    unsigned char joint_angle_data1[18],hand_pos_data1[18],hand_cmd1[6],calib_angle_data1[18];
    double joint_angle[7];
    double pi = 3.1415926;
    double timer;
    vector<double> load;



};

#endif // ZHOU_X_ARM_H
