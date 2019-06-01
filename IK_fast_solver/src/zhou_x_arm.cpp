#include "zhou_x_arm.h"

lz_x_arm::lz_x_arm()
{
    group = new moveit::planning_interface::MoveGroupInterface("x_arm");
    group->setPlanningTime(1);
}

lz_x_arm::~lz_x_arm()
{
    delete group;
}

void lz_x_arm::jointToByteArray()
{
    for(int i=0;i<7;i++ )
    {
    theta=(short int)(joint_angle[i]*1000);
    temp=(unsigned char *)&theta;
    joint_angle_data[4+2*i]=temp[1];joint_angle_data[5+2*i]=temp[0];

    }
}


int lz_x_arm::armReset(unsigned char head, unsigned char tail)
{
    joint_angle_data[0]=head;
    joint_angle_data[1]=tail;
    size_t i=ser_arm.write(joint_angle_data,18);
    cout<<"arm and hand reseted!!!"<<endl;
    return 0;
}

int lz_x_arm::onePoint()
{
    joint_angle[0]=0.8;
    joint_angle[1]=0.46;
    joint_angle[2]=-0.16;
    joint_angle[3]=1.1;
    joint_angle[4]=-0.35;
    joint_angle[5]=0;
    joint_angle[6]=0;

   jointToByteArray();

    size_t i=ser_arm.write(joint_angle_data,18);
    cout<<"trajectory planning data sended to serial: "<<endl;
    return 0;
}

int lz_x_arm::twoPoint(IK_fast_solver::grasp grasp_srv,ros::ServiceClient grasp_client )
{

    ball_robot_sended=ball_robot;
    ball_robot_sended.point.x+=0.01;
    //ball_robot_sended.point.z+=0.015;

    cout<<"ball  pos in robot: "<<ball_robot_sended.point.x<<" "<<ball_robot_sended.point.y<<" "<<ball_robot_sended.point.z<<endl;//unit : m

// step1 calib
    grasp_srv.request.flag=0;
    grasp_srv.request.x=ball_robot_sended.point.x;
    grasp_srv.request.y=ball_robot_sended.point.y;
    grasp_srv.request.z=ball_robot_sended.point.z;
    grasp_srv.request.q_w=0.5;
    grasp_srv.request.q_x=0.5;
    grasp_srv.request.q_y=-0.5;
    grasp_srv.request.q_z=0.5;
    grasp_client.call(grasp_srv);

    cout << "target angle step 1: "\
            << grasp_srv.response.angle0 << " " << grasp_srv.response.angle1 << " "\
            << grasp_srv.response.angle2 << " " << grasp_srv.response.angle3 << " "\
            << grasp_srv.response.angle4 << " " << grasp_srv.response.angle5 << " "\
            << grasp_srv.response.angle6 << endl;

    joint_angle[0]=grasp_srv.response.angle0;
    joint_angle[1]=grasp_srv.response.angle1;
    joint_angle[2]=grasp_srv.response.angle2;
    joint_angle[3]=grasp_srv.response.angle3;
    joint_angle[4]=grasp_srv.response.angle4;
    joint_angle[5]=grasp_srv.response.angle5;
    joint_angle[6]=grasp_srv.response.angle6;

   jointToByteArray();


    for(int i=0;i<18;i++)
   calib_angle_data[i]=joint_angle_data[i];

    cout<<"input something to confirm ball's pos and step1 angle"<<endl;
    cin>>c;

    ser_arm.write(joint_angle_data,18);
    cout<<"data sended to serial: "<<endl;
}

int lz_x_arm::threepoint(IK_fast_solver::grasp grasp_srv,ros::ServiceClient grasp_client)
{
    sleep(6);//sleep 5 seconds to ensure step1 executed(the prerequest of caliberation).
    cout<<"finish sleeping "<<endl;

    geometry_msgs::PointStamped hand_tracker2,hand_robot;//for error detection

    //make up for the error
    cout<<"step 2: compute the error!!!"<<endl;


    //tf::TransformListener listener;
    tf::StampedTransform joint6_robot;

    geometry_msgs::Pose target_pose;

    try
    {
            //joint6_robot is the transform from frame /joint6_link to frame /robot_link.
          tf_listener->lookupTransform("robot_link", "joint6_link",ros::Time(0), joint6_robot);

    grasp_srv.request.flag=1;
    grasp_srv.request.x=joint6_robot.getOrigin().x();
    grasp_srv.request.y=joint6_robot.getOrigin().y();
    grasp_srv.request.z=joint6_robot.getOrigin().z();
    grasp_srv.request.q_w=joint6_robot.getRotation().getW();
    grasp_srv.request.q_x=joint6_robot.getRotation().getX();
    grasp_srv.request.q_y=joint6_robot.getRotation().getY();
    grasp_srv.request.q_z=joint6_robot.getRotation().getZ();
    grasp_client.call(grasp_srv);

            //cout << "real angle step 1: "\
                    << grasp_srv.response.angle0 << " " << grasp_srv.response.angle1 << " "\
                    << grasp_srv.response.angle2 << " " << grasp_srv.response.angle3 << " "\
                    << grasp_srv.response.angle4 << " " << grasp_srv.response.angle5 << " "\
                    << grasp_srv.response.angle6 << endl;

            //cout<<"angle error step1: "<<grasp_srv.response.angle0-grasp_srv.response.angle0<<" "<<grasp_srv.response.angle1-grasp_srv.response.angle1<<" "<<grasp_srv.response.angle2-grasp_srv.response.angle2<<" "<<grasp_srv.response.angle3-grasp_srv.response.angle3<<" "<<grasp_srv.response.angle4-grasp_srv.response.angle4<<" "<<grasp_srv.response.angle5-grasp_srv.response.angle5<<" "<<grasp_srv.response.angle6-grasp_srv.response.angle6<<" "<<endl;
    }
    catch (tf::TransformException &ex)
    {
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
    }

    cout<<"input something to confirm the error"<<endl;
    cin>>c;

     grasp_srv.request.flag=2;
     grasp_srv.request.x=ball_robot_sended.point.x;
     grasp_srv.request.y=ball_robot_sended.point.y;
     grasp_srv.request.z=ball_robot_sended.point.z;
     grasp_srv.request.q_w=0;
     grasp_srv.request.q_x=0;
     grasp_srv.request.q_y=0;
     grasp_srv.request.q_z=0;
     grasp_client.call(grasp_srv);

    cout << "angle step 2: "\
            << grasp_srv.response.angle0 << " " << grasp_srv.response.angle1 << " "\
            << grasp_srv.response.angle2 << " " << grasp_srv.response.angle3 << " "\
            << grasp_srv.response.angle4 << " " << grasp_srv.response.angle5 << " "\
            << grasp_srv.response.angle6 << endl;

    cout<<"step 3: make up for the error!!!"<<endl;

    /*grasp_srv.response.angle0-=(grasp_srv.response.angle0-grasp_srv.response.angle0);
    grasp_srv.response.angle1-=(grasp_srv.response.angle1-grasp_srv.response.angle1);
    grasp_srv.response.angle2-=(grasp_srv.response.angle2-grasp_srv.response.angle2);
    grasp_srv.response.angle3-=(grasp_srv.response.angle3-grasp_srv.response.angle3);
    grasp_srv.response.angle4-=(grasp_srv.response.angle4-grasp_srv.response.angle4);
    grasp_srv.response.angle5-=(grasp_srv.response.angle5-grasp_srv.response.angle5);
    grasp_srv.response.angle6-=(grasp_srv.response.angle6-grasp_srv.response.angle6);*/


    joint_angle[0]=grasp_srv.response.angle0;
    joint_angle[1]=grasp_srv.response.angle1;
    joint_angle[2]=grasp_srv.response.angle2;
    joint_angle[3]=grasp_srv.response.angle3;
    joint_angle[4]=grasp_srv.response.angle4;
    joint_angle[5]=grasp_srv.response.angle5;
    joint_angle[6]=grasp_srv.response.angle6;

   jointToByteArray();


    ser_arm.write(joint_angle_data,18);
    cout<<"data sended to serial: "<<endl;

}

void lz_x_arm::drink_water()
{
    sleep(15);

    joint_angle[0]=-0.04;
    joint_angle[1]=0.46;
    joint_angle[2]=1.41;
    joint_angle[3]=2;
    joint_angle[4]=0.51;
    joint_angle[5]=0.44;
    joint_angle[6]=-0.35;

   jointToByteArray();

    ser_arm.write(joint_angle_data,18);
    cout<<"trajectory planning data sended to serial: "<<endl;
}

void lz_x_arm::addObjector()
{

    ballPose.orientation.w=1;
    ballPose.orientation.x=0;
    ballPose.orientation.y=0;
    ballPose.orientation.z=0;
    ballPose.position.x =0;
    ballPose.position.y =-0.1;
    ballPose.position.z =0.66;

    addBall(planning_scene_interface,"base_link","ball1",ballPose,0.03,0);
}

void lz_x_arm::setTargetPose()
{
    tf2::Vector3 seedPosition(0,0,66);
    tf2::Vector3 seedOritationEuler(-pi/2,pi/2,0);

    myOritation.setEuler(-pi/2,pi/2,0);
    target_pose.orientation.w = myOritation.w();
    target_pose.orientation.x=myOritation.x();
    target_pose.orientation.y = myOritation.y();
    target_pose.orientation.z = myOritation.z();

    target_pose.position.x =double(seedPosition.x())/100.0;
    target_pose.position.y =double(seedPosition.y())/100.0;
    target_pose.position.z = double(seedPosition.z())/100.0;
    group->setPoseTarget(target_pose);
    success = group->plan(my_plan);
    //my_plan.trajectory_.joint_trajectory.points.
    if(success)
    {
        std::cout<<"plan success"<<std::endl;
    }
    else
    {
        std::cout<<"plan faild"<<std::endl;
    }
}

void lz_x_arm::getLoadPlan()
{
    if(!success)
        group->execute(my_plan);
     collisionCheck();
     timer=my_plan.planning_time_;


}
