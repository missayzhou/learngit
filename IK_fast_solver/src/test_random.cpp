#include <line.h>

geometry_msgs::PointStamped ball_lscene,ball_robot,ball_robot_sended;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "x_arm_test_random");
  ros::NodeHandle nh;
  ros::ServiceClient grasp_client = nh.serviceClient<IK_fast_solver::grasp>("grasp");
  IK_fast_solver::grasp grasp_srv;
  
  ros::Subscriber pos_sub = nh.subscribe("scene/left/point", 1000, pos_callback); 
  tf::TransformListener tf_listener;
  ros::Timer timer=nh.createTimer(ros::Duration(1.0),boost::bind(&transformPoint,boost::ref(tf_listener)));

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  x_arm_robot lz_xarm(&nh,&grasp_srv,&grasp_client);

  if(argc==2)
  {
      while(ball_robot.point.x==0)
          ros::spinOnce();//waiting until ball detected
      lz_xarm.input_interface();
      lz_xarm.first_moveToTarget();
      sleep(3);//add the motor-reach-target signal here to continue
      try
      {
          tf::StampedTransform joint6_robot;
          tf_listener.lookupTransform("robot_link", "joint6_link",ros::Time(0), joint6_robot);
          lz_xarm.second_moveToTarget(joint6_robot);
      }
      catch (tf::TransformException &ex)
      {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
      }
      lz_xarm.input_interface();
      lz_xarm.three_moveToTarget();
      sleep(6);
  }
  std::cout<<" send ok"<<std::endl;
  //让机械臂按照规划的轨迹开始运动。
  collisionCheck();
  ros::waitForShutdown();
  return 0;
}
