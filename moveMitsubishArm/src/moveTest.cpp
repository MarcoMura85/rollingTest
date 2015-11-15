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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


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
                                                  0.1,  // eef_step
                                                  0.0,   // jump_threshold
                                                  trajectory_msg, false);

    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), "arm");
    //robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), "manipulator");

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
    std::cout<<"target.y="<<targetPose.position.y<<" pose.y="<<pose_target.position.y<<std::endl;
    std::cout<<"target.z="<<targetPose.position.z<<" pose.z="<<pose_target.position.z<<std::endl;

}


void moveMoveIt (moveit::planning_interface::MoveGroup *group, float dx,float dy,float dz,float rx, float ry,float rz){

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose targetPose = group->getCurrentPose().pose;
    std::vector< double > Angles=group->getCurrentRPY();
    moveit::planning_interface::MoveGroup::Plan plan;

    //waypoints.push_back(targetPose);

    targetPose.position.x += dx;
    targetPose.position.y += dy;
    targetPose.position.z += dz;
    targetPose.orientation = tf::createQuaternionMsgFromRollPitchYaw(Angles[0]+rx,Angles[1]+ry,Angles[2]+rz);

    group->setPoseTarget(targetPose);
    group->plan(plan);
    group->move();

    /*waypoints.push_back(targetPose);

    moveit_msgs::RobotTrajectory trajectory_msg;
    group->setPlanningTime(15.0);


    double fraction = group->computeCartesianPath(waypoints,
                                                  0.001,  // eef_step
                                                  0.0,   // jump_threshold
                                                  trajectory_msg, false);

    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), "arm");
    //robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), "manipulator");

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

    // while(group.getCurrentPose().pose!=targetPose)
//  {
//    std::cout<<"wait"<<std::endl;
//    }
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
    std::cout<<"target.y="<<targetPose.position.y<<" pose.y="<<pose_target.position.y<<std::endl;
    std::cout<<"target.z="<<targetPose.position.z<<" pose.z="<<pose_target.position.z<<std::endl;*/

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

    ros::AsyncSpinner spinner(4);
    spinner.start();

    //moveit::planning_interface::MoveGroup group("manipulator");
    moveit::planning_interface::MoveGroup group("arm");
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
    group.setPoseReferenceFrame("base_link");
    group.setEndEffectorLink("end_effector");

    group.setGoalTolerance(0.000001);
    group.setGoalJointTolerance(0.000001);
    group.setGoalPositionTolerance(0.000001);

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


    joints["j1"] = 0.00134856;
    joints["j2"] = -0.188797;
    joints["j3"] = 2.09629;
    joints["j4"] = 0.00934622;
    joints["j5"] = -0.311011;
    joints["j6"] = 1.56739;

//    joints["joint_1"] = 0.000119189;
//    joints["joint_2"] = -0.0129632;
//    joints["joint_3"] = 0.274897;
//    joints["joint_4"] = 0.00352502;
//    joints["joint_5"] = -0.262732;
//    joints["joint_6"] = 1.56739;

//    joint 0= 0.00134856
//    joint 1= -0.188797
//    joint 2= 2.09629
//    joint 3= 0.00934622
//    joint 4= -0.311011
//    joint 5= 0.00067267


    group.setJointValueTarget(joints);
    group.plan(plan);
    group.move();//*/


    sleep(10);

/*/-------------------------------------------------

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    //joint_values[0] = 1.57;
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    // Check whether any joint is outside its joint limits
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
    // Enforce the joint limits for this state and check again
//    kinematic_state->enforceBounds();
//    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("torque_sensor_joint");
    // Print end-effector pose. Remember that this is in the model frame
    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);
    // Now, we can print out the IK solution (if found):
    if (found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for(std::size_t i=0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }
    else
    {
        ROS_INFO("Did not find IK solution");
    }



//----------------------------------------------------*/


//    moveCartesian (&group, 0, -0.03, 0.0, 0, 0.0, 0.0);
//    sleep(3);
//    moveCartesian (&group, 0, -0.03, 0.0, 0, 0.0, 0.0);
//    sleep(3);
//    moveCartesian (&group, 0, -0.03, 0.0, 0, 0.0, 0.0);
//    sleep(3);
//    moveCartesian (&group, 0, -0.03, 0.0, 0, 0.0, 0.0);
//    sleep(3);


  /*  target_pose = group.getCurrentPose().pose;

    target_pose.position.y += -0.10;
    group.setPoseTarget(target_pose);
    group.plan(plan);
    group.move();
    sleep(10);
    target_pose.position.y += -0.10;
    group.setPoseTarget(target_pose);
    group.plan(plan);
    group.move();
    sleep(10);
    target_pose.position.y += -0.10;
    group.setPoseTarget(target_pose);
    group.plan(plan);
    group.move();
    sleep(10);
    target_pose.position.y += -0.10;
    group.setPoseTarget(target_pose);
    group.plan(plan);
    group.move();
    sleep(10);*/



    //  new cartesian position from keyboard
    ros::Subscriber subscriber = node_handle.subscribe("position", 1000, positionCallback);

    std::cout<<"Waiting for Keyboard inputsMMM"<<std::endl;


    while (ros::ok()){
        //ros::spinOnce();
        if(isNewPosition){
            //		moveCartesian (&group, robotIncrement[0], robotIncrement[1], robotIncrement[2],
            //				robotIncrement[3], robotIncrement[4], robotIncrement[5]);
            moveCartesian (&group, robotIncrement[0], robotIncrement[1], robotIncrement[2],
                    0.0,0.0,0.0);

           // moveMoveIt(&group, robotIncrement[0], robotIncrement[1], robotIncrement[2],
           //            0.0,0.0,0.0);
        }

    }//


    ros::shutdown();
    return 0;
}
