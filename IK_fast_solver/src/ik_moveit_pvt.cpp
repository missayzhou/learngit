#include<zhou_x_arm.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"ik_moveit_node");
    ros::NodeHandle nh;
    ros::ServiceClient grasp_client = nh.serviceClient<IK_fast_solver::grasp>("grasp");
    IK_fast_solver::grasp grasp_srv;
    ros::Subscriber pos_sub = nh.subscribe("scene/left/point", 1000, pos_callback);
    tf::TransformListener tf_listener;
    ros::Timer timer=nh.createTimer(ros::Duration(1.0),boost::bind(&transformPoint,boost::ref(tf_listener)));

    ros::AsyncSpinner spinner(1);
    spinner.start();
    lz_x_arm  lz_arm;
    ros::WallDuration(1.0).sleep();

    lz_arm.armReset();
    lz_arm.addObjector();
    lz_arm.graspedObjectType=atoi(argv[1]);
    lz_arm.tf_listener=&tf_listener;

    if(lz_arm.graspedObjectType==1)
    {
    lz_arm.onePoint();
    lz_arm.twoPoint(grasp_srv,grasp_client);
    lz_arm.threepoint(grasp_srv,grasp_client);
    lz_arm.drink_water();
    }
    else {
        lz_arm.onePoint();
        lz_arm.twoPoint(grasp_srv,grasp_client);
        lz_arm.threepoint(grasp_srv,grasp_client);
    }

    lz_arm.armReset();

}
