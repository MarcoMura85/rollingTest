#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveMitsubishArm/KeyboardInput.h>


void moveCartesian (moveit::planning_interface::MoveGroup *group, float dx,float dy,float dz,float rx, float ry,float rz){

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose targetPose = group->getCurrentPose().pose;
    std::vector< double > Angles=group->getCurrentRPY();
    moveit::planning_interface::MoveGroup::Plan plan;

    waypoints.push_back(targetPose);

    targetPose.position.x += dx;
    targetPose.position.y += dy;
    targetPose.position.z += dz;
    targetPose.orientation = tf::createQuaternionMsgFromRollPitchYaw(Angles[0]+rx,Angles[1]+ry,Angles[2]+rz);

    waypoints.push_back(targetPose);

    moveit_msgs::RobotTrajectory trajectory_msg;
    group->setPlanningTime(5.0);


    double fraction = group->computeCartesianPath(waypoints,
                                                  0.0001,  // eef_step
                                                  0.0,   // jump_threshold
                                                  trajectory_msg, false);

    // First to create a RobotTrajectory object
   // robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), "arm");
    robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), "manipulator");

    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*group->getCurrentState(), trajectory_msg);    

    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory_msg);
    plan.trajectory_ = trajectory_msg;
    //sleep(5.0);

    /* while(group.getCurrentPose().pose!=targetPose)
  {
    std::cout<<"wait"<<std::endl;
    }*/
    //return(plan);

    group->execute(plan);

//    group->setPoseTarget(targetPose);
//    group->plan(plan);
//    group->move();

    std::cout<<"new position reached"<<std::endl;

//    robot_trajectory::RobotTrajectory trajectory(robot_model, move_group_name_);
//    trajectory.setRobotTrajectoryMsg(*(move_group.getCurrentState()),
//    plan.trajectory_);

    moveit::core::RobotState robot_state = rt.getLastWayPoint();
    Eigen::Affine3d eef_transform =
    robot_state.getGlobalLinkTransform(group->getEndEffectorLink());

    geometry_msgs::Pose pose_target;
    tf::poseEigenToMsg(eef_transform, pose_target);

    std::cout<<"target.x="<<targetPose.position.x<<" pose.x="<<pose_target.position.x<<std::endl;
    std::cout<<"target.x="<<targetPose.position.y<<" pose.x="<<pose_target.position.y<<std::endl;
    std::cout<<"target.x="<<targetPose.position.z<<" pose.x="<<pose_target.position.z<<std::endl;

}


float robotIncrement[6]= {0,0,0,0,0,0};
bool isNewPosition = false; 

void positionCallback(const moveMitsubishArm::KeyboardInput& msg)
{
    
    if ( msg.x+msg.y+msg.z+msg.rx+msg.ry+msg.rz!=0 ){
        isNewPosition=true;
        robotIncrement[0]=msg.x;
        robotIncrement[1]=msg.y;
        robotIncrement[2]=msg.z;
        robotIncrement[3]=msg.rx;
        robotIncrement[4]=msg.ry;
        robotIncrement[5]=msg.rz;
        ROS_INFO("New position: %.1f,%.1f,%.1f", msg.x, msg.y, msg.z );
    }
    else
        isNewPosition=false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup group("manipulator");
    //moveit::planning_interface::MoveGroup group("arm");
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

//    std::string eef_link = group.getEndEffectorLink();
//    std::string eef = group.getEndEffector();
//    std::string base_link = group.getPoseReferenceFrame();

    //  std::cout<<eef_link<<std::endl;
    //  std::cout<<base_link<<std::endl;

    moveit::planning_interface::MoveGroup::Plan plan;
    group.setStartStateToCurrentState();
    //group.setPoseReferenceFrame("base_link");
    //group.setEndEffectorLink("end_effector");

    double orientToll = group.getGoalOrientationTolerance();
    double positionToll = group.getGoalPositionTolerance () ;
    double jointToll = group.getGoalJointTolerance() ;

    std::cout<<" orientation toll= "<<orientToll<<std::endl;
    std::cout<<" position tool = "<<positionToll<<std::endl;
    std::cout<<" joint tool = "<<jointToll<<std::endl;


    // debug variables
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = group.getCurrentPose().pose;
    geometry_msgs::Pose init_pose = group.getCurrentPose().pose;


    //initial position of my manipulator
    std::map<std::string, double> joints;
    std::vector<double> jointDbg = group.getCurrentJointValues();

    for (int i=0; i< jointDbg.size(); i++)
        std::cout<<"joint i"<<i<<"= "<<jointDbg[i]<<std::endl;//*/


//    joints["j1"] = 0.00134856;
//    joints["j2"] = -0.188797;
//    joints["j3"] = 2.09629;
//    joints["j4"] = 0.00934622;
//    joints["j5"] = -0.311011;
//    joints["j6"] = 1.56739;

    joints["joint_1"] = 0.000119189;
    joints["joint_2"] = -0.0129632;
    joints["joint_3"] = 0.274897;
    joints["joint_4"] = 0.00352502;
    joints["joint_5"] = -0.262732;
    joints["joint_6"] = 1.56739;

//    joint 0= 0.00134856
//    joint 1= -0.188797
//    joint 2= 2.09629
//    joint 3= 0.00934622
//    joint 4= -0.311011
//    joint 5= 0.00067267


    group.setJointValueTarget(joints);
    group.plan(plan);
    group.move();//*/


    sleep(5);

    //  new cartesian position from keyboard
    ros::Subscriber subscriber = node_handle.subscribe("position", 1000, positionCallback);

//    moveCartesian (&group, 0, -0.03, 0.0, 0, 0.0, 0.0);
//    sleep(10);
//    moveCartesian (&group, 0, -0.03, 0.0, 0, 0.0, 0.0);
//    sleep(10);
//    moveCartesian (&group, 0, -0.03, 0.0, 0, 0.0, 0.0);
//    sleep(10);
//    moveCartesian (&group, 0, -0.03, 0.0, 0, 0.0, 0.0);
//    sleep(10);
    std::cout<<"Waiting for Keyboard inputs"<<std::endl;


    while (ros::ok()){
        if(isNewPosition){
            //		moveCartesian (&group, robotIncrement[0], robotIncrement[1], robotIncrement[2],
            //				robotIncrement[3], robotIncrement[4], robotIncrement[5]);
            moveCartesian (&group, robotIncrement[0], robotIncrement[1], robotIncrement[2],
                    0.0,0.0,0.0);
        }

    }//


    ros::shutdown();
    return 0;
}
