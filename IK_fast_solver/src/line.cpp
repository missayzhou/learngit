#include <line.h>

x_arm_robot::x_arm_robot(ros::NodeHandle* nht,IK_fast_solver::grasp* grasp_srvt,ros::ServiceClient* grasp_clientt)
{
    nh=nht;
    grasp_srv=grasp_srvt;
    grasp_client=grasp_clientt;

    time_delay=10000;
    usb_id= "/dev/ttyUSB0";

    usb_id = nh->getParam("usb_id",usb_id);
    time_delay=nh->getParam("time_delay",time_delay);
    success = new moveit::planning_interface::MoveItErrorCode();
    group = new moveit::planning_interface::MoveGroupInterface("x_arm");
    group->setPlanningTime(1);
    my_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    moveit_arm = new serial::Serial();
    ser_hand = new serial::Serial();
    target_pose= new geometry_msgs::Pose();
    serialPortOpened = false;
    Ser_Arm_Initialize();
}

x_arm_robot::~x_arm_robot()
{
    delete success;
    delete group;
    delete my_plan ;
    delete planning_scene_interface;
    delete moveit_arm;
    delete target_pose;
}

void x_arm_robot::first_moveToTarget()
{
    // *success = group->plan(*my_plan);
    // group->execute(*my_plan);
    ball_robot_sended=ball_robot;
    ball_robot_sended.point.x+=0.01;
    cout<<"ball  pos in robot: "<<ball_robot_sended.point.x<<" "<<ball_robot_sended.point.y<<" "<<ball_robot_sended.point.z<<endl;//unit : m

    // step1 calib
    grasp_srv->request.flag=0;
    grasp_srv->request.x=ball_robot_sended.point.x;
    grasp_srv->request.y=ball_robot_sended.point.y;
    grasp_srv->request.z=ball_robot_sended.point.z;
    grasp_srv->request.q_w=0.5;
    grasp_srv->request.q_x=0.5;
    grasp_srv->request.q_y=-0.5;
    grasp_srv->request.q_z=0.5;
    grasp_client->call(*grasp_srv);

    cout << "target angle step 1: "\
         << grasp_srv->response.angle0 << " " << grasp_srv->response.angle1 << " "\
         << grasp_srv->response.angle2 << " " << grasp_srv->response.angle3 << " "\
         << grasp_srv->response.angle4 << " " << grasp_srv->response.angle5 << " "\
         << grasp_srv->response.angle6 << endl;
    jointValue.push_back(grasp_srv->response.angle0);
    jointValue.push_back(grasp_srv->response.angle1);
    jointValue.push_back(grasp_srv->response.angle2);
    jointValue.push_back(grasp_srv->response.angle3);
    jointValue.push_back(grasp_srv->response.angle4);
    jointValue.push_back(grasp_srv->response.angle5);
    jointValue.push_back(grasp_srv->response.angle6);
    group->setJointValueTarget(jointValue);

    *success = group->plan(*my_plan);
    if(*success)
    {
        std::cout<<"plan success"<<std::endl;
    }
    else
    {
        std::cout<<"plan faild"<<std::endl;
    }
    std::cout<<" "<<my_plan->trajectory_.joint_trajectory.points.at(0)<<std::endl;
    std::cout<<" size:"<<my_plan->trajectory_.joint_trajectory.points.size()<<std::endl;
    send_pvt();
}

void x_arm_robot::second_moveToTarget(tf::StampedTransform & joint6_robot)
{

    grasp_srv->request.flag=1;
    grasp_srv->request.x=joint6_robot.getOrigin().x();
    grasp_srv->request.y=joint6_robot.getOrigin().y();
    grasp_srv->request.z=joint6_robot.getOrigin().z();
    grasp_srv->request.q_w=joint6_robot.getRotation().getW();
    grasp_srv->request.q_x=joint6_robot.getRotation().getX();
    grasp_srv->request.q_y=joint6_robot.getRotation().getY();
    grasp_srv->request.q_z=joint6_robot.getRotation().getZ();
    grasp_client->call(*grasp_srv);

    jointValue.at(0)=2*jointValue.at(0)-grasp_srv->response.angle0;
    jointValue.at(1)=2*jointValue.at(1)-grasp_srv->response.angle1;
    jointValue.at(2)=2*jointValue.at(2)-grasp_srv->response.angle2;
    jointValue.at(3)=2*jointValue.at(3)-grasp_srv->response.angle3;
    jointValue.at(4)=2*jointValue.at(4)-grasp_srv->response.angle4;
    jointValue.at(5)=2*jointValue.at(5)-grasp_srv->response.angle5;
    jointValue.at(6)=2*jointValue.at(6)-grasp_srv->response.angle6;
    group->setJointValueTarget(jointValue);

    *success = group->plan(*my_plan);
    group->execute(*my_plan);

}

void x_arm_robot::three_moveToTarget()
{
    grasp_srv->request.flag=2;
    grasp_srv->request.x=ball_robot_sended.point.x;
    grasp_srv->request.y=ball_robot_sended.point.y;
    grasp_srv->request.z=ball_robot_sended.point.z;
    grasp_srv->request.q_w=0;
    grasp_srv->request.q_x=0;
    grasp_srv->request.q_y=0;
    grasp_srv->request.q_z=0;
    grasp_client->call(*grasp_srv);

    jointValue.at(0)=grasp_srv->response.angle0;
    jointValue.at(1)=grasp_srv->response.angle1;
    jointValue.at(2)=grasp_srv->response.angle2;
    jointValue.at(3)=grasp_srv->response.angle3;
    jointValue.at(4)=grasp_srv->response.angle4;
    jointValue.at(5)=grasp_srv->response.angle5;
    jointValue.at(6)=grasp_srv->response.angle6;
    group->setJointValueTarget(jointValue);
    *success = group->plan(*my_plan);
    send_pvt();
    group->execute(*my_plan);


}
void x_arm_robot::send_pvt()
{
    lz lz_serial;
    unsigned char lz_serial_move[65]={0};

    lz_serial_move[0]='#';
    lz_serial_move[1]='*';
    lz_serial_move[2]='#';
    lz_serial_move[3]='*';
    lz_serial_move[61]='*';
    lz_serial_move[62]='#';
    lz_serial_move[63]='*';
    lz_serial_move[64]='#';
    std::ofstream my_grasp("/home/zhoushuo/catkin_ws/src/x_arm_hand_grasp_planning/src/my_grasp.txt");

    int point_number=my_plan->trajectory_.joint_trajectory.points.size();
    for(int i=0;i<point_number;i++)
    {
        if(i==(point_number-1))
        {
            lz_serial_move[4]=0;
        }
        else {
            lz_serial_move[4]=30;
        }
        for(int j=0;j<7;j++)
        {
            if(j==0)
            {
                lz_serial.temp=my_plan->trajectory_.joint_trajectory.points.at(i).positions.at(j+1)+my_plan->trajectory_.joint_trajectory.points.at(i).positions.at(j);//temp[1] is the higher 8 digits
                lz_serial.temp=lz_serial.temp*57.2958*235/7.5;
                lz_serial_move[5+6*8]=lz_serial.moveit_arm_lz[0];
                lz_serial_move[6+6*8]=lz_serial.moveit_arm_lz[1];
                lz_serial_move[7+6*8]=lz_serial.moveit_arm_lz[2];
                lz_serial_move[8+6*8]=lz_serial.moveit_arm_lz[3];
                my_grasp<<lz_serial.temp<<" ";
                lz_serial.temp=my_plan->trajectory_.joint_trajectory.points.at(i).velocities.at(j+1)+my_plan->trajectory_.joint_trajectory.points.at(i).velocities.at(j);//temp[1] is the higher 8 digits
                lz_serial.temp=lz_serial.temp*235*30/3.1415926;
                lz_serial_move[9+6*8]=lz_serial.moveit_arm_lz[0];
                lz_serial_move[10+6*8]=lz_serial.moveit_arm_lz[1];
                lz_serial_move[11+6*8]=lz_serial.moveit_arm_lz[2];
                lz_serial_move[12+6*8]=lz_serial.moveit_arm_lz[3];
                my_grasp<<lz_serial.temp<<" ";
            }
            if(j==1)
            {
                lz_serial.temp=my_plan->trajectory_.joint_trajectory.points.at(i).positions.at(j)-my_plan->trajectory_.joint_trajectory.points.at(i).positions.at(j-1);//temp[1] is the higher 8 digits
                lz_serial.temp=(-1)*lz_serial.temp*57.2958*200/7.5;
                lz_serial_move[5+5*8]=lz_serial.moveit_arm_lz[0];
                lz_serial_move[6+5*8]=lz_serial.moveit_arm_lz[1];
                lz_serial_move[7+5*8]=lz_serial.moveit_arm_lz[2];
                lz_serial_move[8+5*8]=lz_serial.moveit_arm_lz[3];
                my_grasp<<lz_serial.temp<<" ";
                lz_serial.temp=(-1)*my_plan->trajectory_.joint_trajectory.points.at(i).velocities.at(j)-my_plan->trajectory_.joint_trajectory.points.at(i).velocities.at(j-1);//temp[1] is the higher 8 digits
                lz_serial.temp=lz_serial.temp*200*30/3.1415926;
                lz_serial_move[9+5*8]=lz_serial.moveit_arm_lz[0];
                lz_serial_move[10+5*8]=lz_serial.moveit_arm_lz[1];
                lz_serial_move[11+5*8]=lz_serial.moveit_arm_lz[2];
                lz_serial_move[12+5*8]=lz_serial.moveit_arm_lz[3];
                my_grasp<<lz_serial.temp<<" ";
            }
            if(j==2)
            {
                lz_serial.temp=my_plan->trajectory_.joint_trajectory.points.at(i).positions.at(j);//temp[1] is the higher 8 digits
                lz_serial.temp=lz_serial.temp*57.2958*100/7.5;
                lz_serial_move[5+4*8]=lz_serial.moveit_arm_lz[0];
                lz_serial_move[6+4*8]=lz_serial.moveit_arm_lz[1];
                lz_serial_move[7+4*8]=lz_serial.moveit_arm_lz[2];
                lz_serial_move[8+4*8]=lz_serial.moveit_arm_lz[3];
                my_grasp<<lz_serial.temp<<" ";
                lz_serial.temp=my_plan->trajectory_.joint_trajectory.points.at(i).velocities.at(j);//temp[1] is the higher 8 digits
                lz_serial.temp=lz_serial.temp*100*30/3.1415926;
                lz_serial_move[9+4*8]=lz_serial.moveit_arm_lz[0];
                lz_serial_move[10+4*8]=lz_serial.moveit_arm_lz[1];
                lz_serial_move[11+4*8]=lz_serial.moveit_arm_lz[2];
                lz_serial_move[12+4*8]=lz_serial.moveit_arm_lz[3];
                my_grasp<<lz_serial.temp<<" ";
            }
            if(j==3)
            {
                lz_serial.temp=my_plan->trajectory_.joint_trajectory.points.at(i).positions.at(j)+my_plan->trajectory_.joint_trajectory.points.at(i).positions.at(j+1);//temp[1] is the higher 8 digits
                lz_serial.temp=(-1)*lz_serial.temp*57.2958*235/7.5;
                lz_serial_move[5+3*8]=lz_serial.moveit_arm_lz[0];
                lz_serial_move[6+3*8]=lz_serial.moveit_arm_lz[1];
                lz_serial_move[7+3*8]=lz_serial.moveit_arm_lz[2];
                lz_serial_move[8+3*8]=lz_serial.moveit_arm_lz[3];
                my_grasp<<lz_serial.temp<<" ";
                lz_serial.temp=(-1)*my_plan->trajectory_.joint_trajectory.points.at(i).velocities.at(j)+my_plan->trajectory_.joint_trajectory.points.at(i).velocities.at(j+1);//temp[1] is the higher 8 digits
                lz_serial.temp=lz_serial.temp*235*30/3.1415926;
                lz_serial_move[9+3*8]=lz_serial.moveit_arm_lz[0];
                lz_serial_move[10+3*8]=lz_serial.moveit_arm_lz[1];
                lz_serial_move[11+3*8]=lz_serial.moveit_arm_lz[2];
                lz_serial_move[12+3*8]=lz_serial.moveit_arm_lz[3];
                my_grasp<<lz_serial.temp<<" ";
            }
            if(j==4)
            {
                lz_serial.temp=my_plan->trajectory_.joint_trajectory.points.at(i).positions.at(j-1)-my_plan->trajectory_.joint_trajectory.points.at(i).positions.at(j);//temp[1] is the higher 8 digits
                lz_serial.temp=lz_serial.temp*57.2958*200/7.5;
                lz_serial_move[5+2*8]=lz_serial.moveit_arm_lz[0];
                lz_serial_move[6+2*8]=lz_serial.moveit_arm_lz[1];
                lz_serial_move[7+2*8]=lz_serial.moveit_arm_lz[2];
                lz_serial_move[8+2*8]=lz_serial.moveit_arm_lz[3];
                my_grasp<<lz_serial.temp<<" ";
                lz_serial.temp=my_plan->trajectory_.joint_trajectory.points.at(i).velocities.at(j-1)-my_plan->trajectory_.joint_trajectory.points.at(i).velocities.at(j);//temp[1] is the higher 8 digits
                lz_serial.temp=lz_serial.temp*200*30/3.1415926;
                lz_serial_move[9+2*8]=lz_serial.moveit_arm_lz[0];
                lz_serial_move[10+2*8]=lz_serial.moveit_arm_lz[1];
                lz_serial_move[11+2*8]=lz_serial.moveit_arm_lz[2];
                lz_serial_move[12+2*8]=lz_serial.moveit_arm_lz[3];
                my_grasp<<lz_serial.temp<<" ";
            }

            if(j==5)
            {
                lz_serial.temp=my_plan->trajectory_.joint_trajectory.points.at(i).positions.at(j)+my_plan->trajectory_.joint_trajectory.points.at(i).positions.at(j+1);//temp[1] is the higher 8 digits
                lz_serial.temp=lz_serial.temp*57.2958*462/7.5;
                lz_serial_move[5+1*8]=lz_serial.moveit_arm_lz[0];
                lz_serial_move[6+1*8]=lz_serial.moveit_arm_lz[1];
                lz_serial_move[7+1*8]=lz_serial.moveit_arm_lz[2];
                lz_serial_move[8+1*8]=lz_serial.moveit_arm_lz[3];
                my_grasp<<lz_serial.temp<<" ";
                lz_serial.temp=my_plan->trajectory_.joint_trajectory.points.at(i).velocities.at(j)+my_plan->trajectory_.joint_trajectory.points.at(i).velocities.at(j+1);//temp[1] is the higher 8 digits
                lz_serial.temp=lz_serial.temp*462*30/3.1415926;
                lz_serial_move[9+1*8]=lz_serial.moveit_arm_lz[0];
                lz_serial_move[10+1*8]=lz_serial.moveit_arm_lz[1];
                lz_serial_move[11+1*8]=lz_serial.moveit_arm_lz[2];
                lz_serial_move[12+1*8]=lz_serial.moveit_arm_lz[3];
                my_grasp<<lz_serial.temp<<" ";
            }
            if(j==6)
            {
                lz_serial.temp=(-1)*my_plan->trajectory_.joint_trajectory.points.at(i).positions.at(j-1)-my_plan->trajectory_.joint_trajectory.points.at(i).positions.at(j);//temp[1] is the higher 8 digits
                lz_serial.temp=lz_serial.temp*57.2958*462/7.5;
                lz_serial_move[5]=lz_serial.moveit_arm_lz[0];
                lz_serial_move[6]=lz_serial.moveit_arm_lz[1];
                lz_serial_move[7]=lz_serial.moveit_arm_lz[2];
                lz_serial_move[8]=lz_serial.moveit_arm_lz[3];
                my_grasp<<lz_serial.temp<<" ";
                lz_serial.temp=(-1)*my_plan->trajectory_.joint_trajectory.points.at(i).velocities.at(j-1)-my_plan->trajectory_.joint_trajectory.points.at(i).velocities.at(j);//temp[1] is the higher 8 digits
                lz_serial.temp=lz_serial.temp*462*30/3.1415926;
                lz_serial_move[9]=lz_serial.moveit_arm_lz[0];
                lz_serial_move[10]=lz_serial.moveit_arm_lz[1];
                lz_serial_move[11]=lz_serial.moveit_arm_lz[2];
                lz_serial_move[12]=lz_serial.moveit_arm_lz[3];
                my_grasp<<lz_serial.temp<<" ";
            }

            // my_grasp<<my_plan.trajectory_.joint_trajectory.points.at(i).positions.at(j)<<"   "<<my_plan.trajectory_.joint_trajectory.points.at(i).velocities.at(j)<<"  ";
            //std::this_thread::sleep_for(std::chrono::microseconds(20));
        }

        my_grasp<<std::endl;
        moveit_arm->write(lz_serial_move,65);
        usleep(time_delay);
    }
    sleep(2);
    my_grasp.close();
}

void x_arm_robot::Ser_Arm_Initialize()
{
    try
    {
        moveit_arm->setPort(usb_id);
        moveit_arm->setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        moveit_arm->setTimeout(to);
        moveit_arm->open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port for Arm");
        serialPortOpened = false;
        return;
    }
    if(moveit_arm->isOpen())
    {
        ROS_INFO_STREAM("Serial_Arm Port initialized");
        serialPortOpened = true;
    }
    else
    {
        serialPortOpened = false;
        return;
    }
}



void x_arm_robot::set_joint_angle(unsigned char* joint_angle_data,IK_fast_solver::grasp grasp_srv)
{
    theta=(short int)(grasp_srv.response.angle0*1000);//plus 1000 to reserve three digit after the dot.
    temp=(unsigned char *)&theta;//temp[1] is the higher 8 digits
    *(joint_angle_data+4)=temp[1];  *(joint_angle_data+5)=temp[0];

    theta=(short int)(grasp_srv.response.angle1*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+6)=temp[1];  *(joint_angle_data+7)=temp[0];

    theta=(short int)(grasp_srv.response.angle2*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+8)=temp[1];  *(joint_angle_data+9)=temp[0];

    theta=(short int)(grasp_srv.response.angle3*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+10)=temp[1];  *(joint_angle_data+11)=temp[0];

    theta=(short int)(grasp_srv.response.angle4*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+12)=temp[1];  *(joint_angle_data+13)=temp[0];

    theta=(short int)(grasp_srv.response.angle5*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+14)=temp[1];  *(joint_angle_data+15)=temp[0];

    theta=(short int)(grasp_srv.response.angle6*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+16)=temp[1];  *(joint_angle_data+17)=temp[0];
}

void x_arm_robot::set_joint_angle_const(unsigned char* joint_angle_data,float angle0,float angle1,float angle2,float angle3,float angle4,float angle5,float angle6)
{
    theta=(short int)(angle0*1000);//plus 1000 to reserve three digit after the dot.
    temp=(unsigned char *)&theta;//temp[1] is the higher 8 digits
    *(joint_angle_data+4)=temp[1];  *(joint_angle_data+5)=temp[0];

    theta=(short int)(angle1*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+6)=temp[1];  *(joint_angle_data+7)=temp[0];

    theta=(short int)(angle2*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+8)=temp[1];  *(joint_angle_data+9)=temp[0];

    theta=(short int)(angle3*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+10)=temp[1];  *(joint_angle_data+11)=temp[0];

    theta=(short int)(angle4*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+12)=temp[1];  *(joint_angle_data+13)=temp[0];

    theta=(short int)(angle5*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+14)=temp[1];  *(joint_angle_data+15)=temp[0];

    theta=(short int)(angle6*1000);
    temp=(unsigned char *)&theta;
    *(joint_angle_data+16)=temp[1];  *(joint_angle_data+17)=temp[0];
}


void x_arm_robot::Ser_Hand_Initialize()
{
    try
    {
        ser_hand->setPort("/dev/ttyUSB0");
        ser_hand->setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser_hand->setTimeout(to);
        ser_hand->open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port for Hand");
        return;
    }
    if(ser_hand->isOpen())
    {
        ROS_INFO_STREAM("Serial_Hand Port initialized");
    }
    else
    {
        return;
    }
}

void x_arm_robot::input_interface()
{
    char c1;
    cout<<"do you want a motion again? y"<<endl;
    cin>>c1;
          while(c!='y')
    {
        cout<<"input error"<<endl;
        cin>>c1;
    }
 }


void pos_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    ball_lscene=*msg;
}

void transformPoint(const tf::TransformListener &listener)
{
    try
    {
        listener.transformPoint("robot_link",ball_lscene,ball_robot);//coordinate transform robot_link
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("exception aroused while coordinate transform!!!");
    }
    //cout<<"ball in robot: "<<endl<<ball_robot<<endl;
}

