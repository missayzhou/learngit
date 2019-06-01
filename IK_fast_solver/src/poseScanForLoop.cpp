#include <ros/ros.h>

#include "addObject.h"
#include "collisionCheck.h"
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>

#include <string>
#include <opencv2/core/core.hpp>

#define X_center 70
#define Y_center 70
#define Z_center 0
#define X_range 140
#define Y_range 140
#define Z_range 70
#define angleSearchN 1
#define angleSearchStepDegree 5
#define pi 3.1415926
#define armLength 66 //cm

int X_lower_limit=0;
int X_upper_limit=40;
int Y_lower_limit=0;
int Y_upper_limit=50;
int Z_lower_limit=10;
int Z_upper_limit=70;

int solvedPointMaxX=-100;
int solvedPointMinX=100;
int solvedPointMaxY=-100;
int solvedPointMinY=100;
int solvedPointMaxZ=-100;
int solvedPointMinZ=100;

int solvedPoints = 0;
long long int solvedIndex = 0;

double jointLoss(moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    double joint_center[]={0.525, 1.22, 0.35,  1.135, 0,    0,    0}; //joint center
    double joint_range[]= {2.09,  2.78, 1.83,  2.27,  3.14, 3.14, 0.52};//joint range
    double loss=0;
    for(int i = 0; i <=6 ; i++)
    {
        loss+= fabs(joint_center[i] - plan.trajectory_.joint_trajectory.points.back().positions.at(i))/joint_range[i];
    }
    return loss;
}

bool onePositionPoseOpti(cv::Mat& positionMatrix,
                      cv::Point3i& seedPoint,
                      cv::Point3d& seedOritationEuler,
                      moveit::planning_interface::MoveGroupInterface& group)
{
    if(sqrt(seedPoint.x*seedPoint.x+seedPoint.y*seedPoint.y+seedPoint.z*seedPoint.z)>armLength)
    {
        int idx[]={seedPoint.x+X_center,seedPoint.y+Y_center,seedPoint.z+Z_center,3};
        positionMatrix.at<double>(idx)=-1;
        return false;
    }

    int idx0[]={seedPoint.x+X_center,seedPoint.y+Y_center,seedPoint.z+Z_center,3};
    if(fabs(positionMatrix.at<double>(idx0)-1)<0.0001)
        return false;


    moveit::planning_interface::MoveItErrorCode success;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::Pose target_pose;
    tf2::Quaternion currentOritation;

    double yaw,pitch,roll;
    double yawOptimal,pitchOptimal,rollOptimal;
    bool reachable=false;

    yaw = seedOritationEuler.x;
    pitch = seedOritationEuler.y;
    roll = seedOritationEuler.z;

    double optiLoss = 10000;  //minimum this loss
    for(int i =-angleSearchN;i<=angleSearchN;i++)
        for(int j =-angleSearchN;j<=angleSearchN;j++)
            for(int k =-angleSearchN;k<=angleSearchN;k++)
            {
                currentOritation.setEuler(yaw+i*angleSearchStepDegree*pi/180,
                                          pitch+j*angleSearchStepDegree*pi/180,
                                          roll+k*angleSearchStepDegree*pi/180);
                target_pose.orientation.w = currentOritation.w();
                target_pose.orientation.x=currentOritation.x();
                target_pose.orientation.y = currentOritation.y();
                target_pose.orientation.z = currentOritation.z();
                target_pose.position.x =double(seedPoint.x)/100.0;
                target_pose.position.y = double(seedPoint.y)/100.0;
                target_pose.position.z = double(seedPoint.z)/100.0;

                group.setPoseTarget(target_pose);
                success = group.plan(my_plan);

                if(success)
                {
                   if(jointLoss(my_plan)<optiLoss)
                   {
                       optiLoss =jointLoss(my_plan);

                       yawOptimal = yaw+i*angleSearchStepDegree*pi/180;
                       pitchOptimal = pitch+j*angleSearchStepDegree*pi/180;
                       rollOptimal = roll+k*angleSearchStepDegree*pi/180;

                       int idx[]={seedPoint.x+X_center,seedPoint.y+Y_center,seedPoint.z+Z_center,0};
                       positionMatrix.at<double>(idx)=yawOptimal;
                       idx[3]=1;
                       positionMatrix.at<double>(idx)=pitchOptimal;
                       idx[3]=2;
                       positionMatrix.at<double>(idx)=rollOptimal;
                       idx[3]=3;
                       positionMatrix.at<double>(idx)=1;
                       reachable =true;
                   }
                }
            }
    solvedPoints++;
    solvedIndex++;
    if(!reachable)
    {
        std::cout<<"solved index:"<<solvedIndex<<", plan fail, point(x,y,z): "<< seedPoint.x <<" "<<seedPoint.y <<" "<<seedPoint.z <<" "<<std::endl;
        return false;
    }
    std::cout<<"solved index:"<<solvedIndex<<", plan success, point(x,y,z): "<< seedPoint.x <<" "<<seedPoint.y <<" "<<seedPoint.z <<" "
            <<"oritation(yaw,pitch,roll): "<< yawOptimal<<" "<<pitchOptimal<<" "<<rollOptimal <<std::endl;
    return true;
}

bool poseCandidateSelect(cv::Mat& positionMatrix,
                         cv::Point3i& seedPoint,
                         cv::Point3d& seedOritationEulerSelected,
                         moveit::planning_interface::MoveGroupInterface& group)
{
    double poseLoss = 10000;
    moveit::planning_interface::MoveItErrorCode success;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::Pose target_pose;
    tf2::Quaternion currentOritation;
    bool selectFlag = false;

    int idx0[]={seedPoint.x+X_center,seedPoint.y+Y_center,seedPoint.z+Z_center,3};
    if(fabs(positionMatrix.at<double>(idx0)-1)<0.0001)
        return false;

    for(int i =-1;i<=1;i++)
        for(int j =-1;j<=1;j++)
            for(int k =-1;k<=1;k++)
            {
                cv::Point3i subSeedPoint(seedPoint.x+i,seedPoint.y+j,seedPoint.z+k);
                int idSub[]={subSeedPoint.x+X_center,subSeedPoint.y+Y_center,subSeedPoint.z+Z_center,3};
                if(positionMatrix.at<double>(idSub)>0 &&
                        subSeedPoint.x + X_center >= 0 && subSeedPoint.x + X_center < X_range &&
                        subSeedPoint.y + Y_center >= 0 && subSeedPoint.y + Y_center < Y_range &&
                        subSeedPoint.z + Z_center >= 0 && subSeedPoint.z + Z_center < Z_range )
                {
                    cv::Point3d tempEuler;
                    idSub[3]=0;
                    tempEuler.x = positionMatrix.at<double>(idSub);
                    idSub[3]=1;
                    tempEuler.y = positionMatrix.at<double>(idSub);
                    idSub[3]=2;
                    tempEuler.z = positionMatrix.at<double>(idSub);

                    currentOritation.setEuler(tempEuler.x,tempEuler.y,tempEuler.z);
                    target_pose.orientation.w = currentOritation.w();
                    target_pose.orientation.x = currentOritation.x();
                    target_pose.orientation.y = currentOritation.y();
                    target_pose.orientation.z = currentOritation.z();
                    target_pose.position.x =double(seedPoint.x)/100.0;
                    target_pose.position.y = double(seedPoint.y)/100.0;
                    target_pose.position.z = double(seedPoint.z)/100.0;

                    group.setPoseTarget(target_pose);
                    success = group.plan(my_plan);

                    if(success)
                    {
                       tf2::Matrix3x3 oritationMatrix(currentOritation);
                       if(jointLoss(my_plan)<poseLoss)
                       {
                           poseLoss = jointLoss(my_plan);
                           seedOritationEulerSelected.x = tempEuler.x;
                           seedOritationEulerSelected.y = tempEuler.y;
                           seedOritationEulerSelected.z = tempEuler.z;
                           selectFlag = true;
                       }
                    }
                }
            }
    return selectFlag;
}


bool positionPoseScan(cv::Mat& positionMatrix,
                             cv::Mat& neighbourMatrix,
                             cv::Point3i& seedPoint,
                             cv::Point3d& seedOritationEuler,
                             moveit::planning_interface::MoveGroupInterface& group,
                             std::string& fileName)
{
    std::vector<std::vector<cv::Point3i>> pointsWithNNeighboursList(27);
    std::vector<std::vector<cv::Point3i>>::iterator it;
    for(it=pointsWithNNeighboursList.begin();it!=pointsWithNNeighboursList.end();it++)
        (*it).clear();  //clear the seed points lists with 0-26 neighbours

    if(onePositionPoseOpti(positionMatrix,seedPoint,seedOritationEuler,group))
    {
        for(int i =-1;i<=1;i++)
            for(int j =-1;j<=1;j++)
                for(int k =-1;k<=1;k++)
                {
                    cv::Point3i subSeedPoint(seedPoint.x+i,seedPoint.y+j,seedPoint.z+k);
                    int idSub[]={subSeedPoint.x+X_center,subSeedPoint.y+Y_center,subSeedPoint.z+Z_center};
                    if(i==0 && j==0 && k==0)
                        neighbourMatrix.at<unsigned char>(idSub)=0;  //no neighbours after pose optimization success
                    else
                    {
                        int idSub1[]={subSeedPoint.x+X_center,subSeedPoint.y+Y_center,subSeedPoint.z+Z_center,3};

                        if(fabs(positionMatrix.at<double>(idSub1)-0)<0.001 &&
                                subSeedPoint.x + X_center >= 0 && subSeedPoint.x + X_center < X_range &&
                                subSeedPoint.y + Y_center >= 0 && subSeedPoint.y + Y_center < Y_range &&
                                subSeedPoint.z + Z_center >= 0 && subSeedPoint.z + Z_center < Z_range &&
                                subSeedPoint.x >= X_lower_limit && subSeedPoint.x <= X_upper_limit &&
                                subSeedPoint.y >= Y_lower_limit && subSeedPoint.y <= Y_upper_limit &&
                                subSeedPoint.z >= Z_lower_limit && subSeedPoint.z <= Z_upper_limit &&
                                sqrt(subSeedPoint.x*subSeedPoint.x+subSeedPoint.y*subSeedPoint.y+subSeedPoint.z*subSeedPoint.z)<=armLength)
                        {
                            neighbourMatrix.at<unsigned char>(idSub)+=1;
                            pointsWithNNeighboursList.at(neighbourMatrix.at<unsigned char>(idSub)).push_back(subSeedPoint);
                        }
                        if(fabs(positionMatrix.at<double>(idSub1)-1)<0.001 &&
                                subSeedPoint.x + X_center >= 0 && subSeedPoint.x + X_center < X_range &&
                                subSeedPoint.y + Y_center >= 0 && subSeedPoint.y + Y_center < Y_range &&
                                subSeedPoint.z + Z_center >= 0 && subSeedPoint.z + Z_center < Z_range &&
                                subSeedPoint.x >= X_lower_limit && subSeedPoint.x <= X_upper_limit &&
                                subSeedPoint.y >= Y_lower_limit && subSeedPoint.y <= Y_upper_limit &&
                                subSeedPoint.z >= Z_lower_limit && subSeedPoint.z <= Z_upper_limit &&
                                sqrt(subSeedPoint.x*subSeedPoint.x+subSeedPoint.y*subSeedPoint.y+subSeedPoint.z*subSeedPoint.z)<=armLength)
                            neighbourMatrix.at<unsigned char>(idSub)=0;
                    }
                }
    }
    else
    {
        std::cout<<"seed point initialized error"<<std::endl;
        return false;
    }


    //show points who have neighbours
//    for(int i =0;i<27;i++)
//    {
//        if(pointsWithNNeighboursList.at(i).size())
//        {
//            std::cout<<i<<" neighbours point: "<<pointsWithNNeighboursList.at(i).size()<<std::endl;
//            for(int j=0;j<pointsWithNNeighboursList.at(i).size();j++)
//                std::cout<<pointsWithNNeighboursList.at(i).at(j)<<std::endl;
//        }
//    }


    int i =27;   //vector index
    for(it=pointsWithNNeighboursList.end()-1;it!=pointsWithNNeighboursList.begin();)
    {
        i--;
        if((*it).size()!=0)
        {
            std::cout<<"vector index(number of the neighbours one point has): "<<i<<" size: "<<(*it).size()<<std::endl;
            cv::Point3d EulerSelected;
            cv::Point3i currentSeedPoint(*((*it).end()-1));
            std::cout<<"currentSeedPoint: "<<currentSeedPoint<<std::endl;
            if(poseCandidateSelect(positionMatrix,currentSeedPoint,EulerSelected,group))
            {
                if(onePositionPoseOpti(positionMatrix,currentSeedPoint,EulerSelected,group))
                {
                    (*it).pop_back();
                    it = pointsWithNNeighboursList.end()-1;
                    i=27;

                    for(int i =-1;i<=1;i++)
                        for(int j =-1;j<=1;j++)
                            for(int k =-1;k<=1;k++)
                            {
                                cv::Point3i subSeedPoint(currentSeedPoint.x+i,currentSeedPoint.y+j,currentSeedPoint.z+k);
                                int idSub[]={subSeedPoint.x+X_center,subSeedPoint.y+Y_center,subSeedPoint.z+Z_center};
                                if(i==0 && j==0 && k==0)
                                    neighbourMatrix.at<unsigned char>(idSub)=0;
                                else
                                {
                                    int idSub1[]={subSeedPoint.x+X_center,subSeedPoint.y+Y_center,subSeedPoint.z+Z_center,3};

                                    if(fabs(positionMatrix.at<double>(idSub1)-0)<0.001 &&
                                            subSeedPoint.x + X_center >= 0 && subSeedPoint.x + X_center < X_range &&
                                            subSeedPoint.y + Y_center >= 0 && subSeedPoint.y + Y_center < Y_range &&
                                            subSeedPoint.z + Z_center >= 0 && subSeedPoint.z + Z_center < Z_range &&
                                            subSeedPoint.x >= X_lower_limit && subSeedPoint.x <= X_upper_limit &&
                                            subSeedPoint.y >= Y_lower_limit && subSeedPoint.y <= Y_upper_limit &&
                                            subSeedPoint.z >= Z_lower_limit && subSeedPoint.z <= Z_upper_limit &&
                                            sqrt(subSeedPoint.x*subSeedPoint.x+subSeedPoint.y*subSeedPoint.y+subSeedPoint.z*subSeedPoint.z)<=armLength)
                                    {
                                        neighbourMatrix.at<unsigned char>(idSub)+=1;
                                        pointsWithNNeighboursList.at(neighbourMatrix.at<unsigned char>(idSub)).push_back(subSeedPoint);
                                    }
                                    if(fabs(positionMatrix.at<double>(idSub1)-1)<0.001 &&
                                            subSeedPoint.x + X_center >= 0 && subSeedPoint.x + X_center < X_range &&
                                            subSeedPoint.y + Y_center >= 0 && subSeedPoint.y + Y_center < Y_range &&
                                            subSeedPoint.z + Z_center >= 0 && subSeedPoint.z + Z_center < Z_range &&
                                            subSeedPoint.x >= X_lower_limit && subSeedPoint.x <= X_upper_limit &&
                                            subSeedPoint.y >= Y_lower_limit && subSeedPoint.y <= Y_upper_limit &&
                                            subSeedPoint.z >= Z_lower_limit && subSeedPoint.z <= Z_upper_limit &&
                                            sqrt(subSeedPoint.x*subSeedPoint.x+subSeedPoint.y*subSeedPoint.y+subSeedPoint.z*subSeedPoint.z)<=armLength)
                                        neighbourMatrix.at<unsigned char>(idSub)=0;
                                }
                            }

                    if(currentSeedPoint.x>solvedPointMaxX)
                        solvedPointMaxX=currentSeedPoint.x;
                    if(currentSeedPoint.x<solvedPointMinX)
                        solvedPointMinX=currentSeedPoint.x;
                    if(currentSeedPoint.y>solvedPointMaxY)
                        solvedPointMaxY=currentSeedPoint.y;
                    if(currentSeedPoint.y<solvedPointMinY)
                        solvedPointMinY=currentSeedPoint.y;
                    if(currentSeedPoint.z>solvedPointMaxZ)
                        solvedPointMaxZ=currentSeedPoint.z;
                    if(currentSeedPoint.z<solvedPointMinZ)
                        solvedPointMinZ=currentSeedPoint.z;
                    //save temp result

                    if(solvedPoints>50)
                    {
                        cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
                        fs<<"solvedPointMaxX"<<solvedPointMaxX;
                        fs<<"solvedPointMinX"<<solvedPointMinX;
                        fs<<"solvedPointMaxY"<<solvedPointMaxY;
                        fs<<"solvedPointMinY"<<solvedPointMinY;
                        fs<<"solvedPointMaxZ"<<solvedPointMaxZ;
                        fs<<"solvedPointMinZ"<<solvedPointMinZ;
                        fs<<"X_lower_limit"<<X_lower_limit;
                        fs<<"X_upper_limit"<<X_upper_limit;
                        fs<<"Y_lower_limit"<<Y_lower_limit;
                        fs<<"Y_upper_limit"<<Y_upper_limit;
                        fs<<"Z_lower_limit"<<Z_lower_limit;
                        fs<<"Z_upper_limit"<<Z_upper_limit;
                        fs<<"positionMatrix"<<positionMatrix;
                        fs.release();
                        solvedPoints = 0;
                    }
                }
            }
            else
            {
                (*it).pop_back();
                it = pointsWithNNeighboursList.end()-1;
                i = 27;
            }
        }
        else
            it--;
    }
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "x_arm_test_random");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveItErrorCode success;

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::MoveGroupInterface group("x_arm");
  group.setPlanningTime(0.1);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  geometry_msgs::Pose ballPose;
  ballPose.orientation.w=1;
  ballPose.orientation.x=0;
  ballPose.orientation.y=0;
  ballPose.orientation.z=0;
  ballPose.position.x =1;
  ballPose.position.y =1;
  ballPose.position.z =1;
  
  std::vector<double> jointValue;
  jointValue.push_back(0.1);
  jointValue.push_back(0.1);
  jointValue.push_back(0.1);
  jointValue.push_back(0.1);
  jointValue.push_back(0.1);
  jointValue.push_back(0.1);
  jointValue.push_back(0.1);
  
  group.setJointValueTarget(jointValue);
          
          
  addBall(planning_scene_interface,"base_link","ball1",ballPose,0.04,0);

  std::string fileName="poseSolve.YAML";


  //generate poteitial oritation
  cv::Point3i seedPosition(0,0,66);
  cv::Point3d seedOritationEuler(-pi/2,pi/2,0);

  int sz[]={X_range,Y_range,Z_range,4}; //4: yaw pitch roll reachable[0,1]
  int nsz[]={X_range,Y_range,Z_range}; //
  cv::Mat positionMatrix(4,sz,CV_64F,cv::Scalar(0));
  cv::Mat neighbourMatrix(3,nsz,CV_8U,cv::Scalar(0));



  positionPoseScan(positionMatrix,neighbourMatrix,seedPosition,seedOritationEuler,group,fileName);
  cv::FileStorage fs;
  fs.open(fileName,cv::FileStorage::WRITE);
  fs<<"solvedPointMaxX"<<solvedPointMaxX;
  fs<<"solvedPointMinX"<<solvedPointMinX;
  fs<<"solvedPointMaxY"<<solvedPointMaxY;
  fs<<"solvedPointMinY"<<solvedPointMinY;
  fs<<"solvedPointMaxZ"<<solvedPointMaxZ;
  fs<<"solvedPointMinZ"<<solvedPointMinZ;
  fs<<"X_lower_limit"<<X_lower_limit;
  fs<<"X_upper_limit"<<X_upper_limit;
  fs<<"Y_lower_limit"<<Y_lower_limit;
  fs<<"Y_upper_limit"<<Y_upper_limit;
  fs<<"Z_lower_limit"<<Z_lower_limit;
  fs<<"Z_upper_limit"<<Z_upper_limit;
  fs<<"positionMatrix"<<positionMatrix;
  fs.release();



  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   

  //让机械臂按照规划的轨迹开始运动。
  if(!success)
      group.execute(my_plan);

  collisionCheck();

  ros::waitForShutdown();
  return 0;
}

