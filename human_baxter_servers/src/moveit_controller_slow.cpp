#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/RobotState.h>
#include "human_baxter_servers/BaxterTrajectory.h"
#include <moveit/robot_trajectory/robot_trajectory.h>

//#include <moveit_visual_tools/moveit_visual_tools.h>
#include <chrono>

#include "human_baxter_servers/PickPlace.h"
#include <geometry_msgs/Point.h>
#include <string>

using namespace std::chrono;


moveit_msgs::RobotState init_baxter_state_r;
moveit_msgs::RobotState init_baxter_state_l;

static const std::string PLANNING_GROUP_R = "right_arm";
static const std::string PLANNING_GROUP_L = "left_arm";

const double jump_threshold = 0.0;
const double eef_step = 0.01;

moveit::planning_interface::MoveGroupInterface *move_group_interface_r;
moveit::planning_interface::MoveGroupInterface *move_group_interface_l;


      
// the plan object to store the trajectory plan in 
moveit::planning_interface::MoveGroupInterface::Plan my_plan;


// to store the success state of the planner
bool success;


// to store the updated joint positions after each trajectory
std::vector<double> joint_group_positions;


// to store the planned trajectory 
moveit_msgs::RobotTrajectory trajectory;


// to store baxter trajectory
human_baxter_servers::BaxterTrajectory my_trajectory;


// Target poses for each of the four trajectories of baxter
geometry_msgs::Pose target_pose1;
geometry_msgs::Pose target_pose2;
geometry_msgs::Pose target_pose3;
geometry_msgs::Pose target_pose4;


// The baxter trajectory publisher
ros::Publisher baxter_trajectory_pub;


//the service advertiser for picking and placing object
ros::ServiceServer pick_server;


// Initializing the move group interfaces for both right and left baxter arms
void setup(){
move_group_interface_r = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_R);
move_group_interface_l = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_L);

}


// Initializing the baxter initial joints positions
void init_baxter_state(){

  init_baxter_state_r.joint_state.name.push_back("right_s0");
  init_baxter_state_r.joint_state.name.push_back("right_s1");
  init_baxter_state_r.joint_state.name.push_back("right_e0");
  init_baxter_state_r.joint_state.name.push_back("right_e1");
  init_baxter_state_r.joint_state.name.push_back("right_w0");
  init_baxter_state_r.joint_state.name.push_back("right_w1");
  init_baxter_state_r.joint_state.name.push_back("right_w2");
  init_baxter_state_r.joint_state.position.push_back(0.52);
  init_baxter_state_r.joint_state.position.push_back(-1.22);
  init_baxter_state_r.joint_state.position.push_back(0.0);
  init_baxter_state_r.joint_state.position.push_back(1.73);
  init_baxter_state_r.joint_state.position.push_back(0.0);  
  init_baxter_state_r.joint_state.position.push_back(0.75);
  init_baxter_state_r.joint_state.position.push_back(0.0);
  
  
  init_baxter_state_l.joint_state.name.push_back("left_s0");
  init_baxter_state_l.joint_state.name.push_back("left_s1");
  init_baxter_state_l.joint_state.name.push_back("left_e0");
  init_baxter_state_l.joint_state.name.push_back("left_e1");
  init_baxter_state_l.joint_state.name.push_back("left_w0");
  init_baxter_state_l.joint_state.name.push_back("left_w1");
  init_baxter_state_l.joint_state.name.push_back("left_w2");
  init_baxter_state_l.joint_state.position.push_back(-0.52);
  init_baxter_state_l.joint_state.position.push_back(-1.22);
  init_baxter_state_l.joint_state.position.push_back(0.0);
  init_baxter_state_l.joint_state.position.push_back(1.73);
  init_baxter_state_l.joint_state.position.push_back(0.0);  
  init_baxter_state_l.joint_state.position.push_back(0.75);
  init_baxter_state_l.joint_state.position.push_back(0.0);
  
  

}


//////////////////////////////////////////////////////////////////
//**************************************************************//
//////////////////////////////////////////////////////////////////

void init_move_group_param(){

  move_group_interface_r->setPlanningTime(1);
  move_group_interface_l->setPlanningTime(1);
  
  //Setting the table borders as the workspace limits
  double minx, miny, minz, maxx, maxy, maxz;
  minx = 0;
  miny = -1;
  minz = 0;
  maxx = 1;
  maxy = 1;
  maxz = 1.5;
  move_group_interface_r->setWorkspace(minx, miny, minz, maxx, maxy, maxz);
  move_group_interface_l->setWorkspace(minx, miny, minz, maxx, maxy, maxz);
  
  //Start state monitoring for faster retrieval of the robot states
  move_group_interface_r->startStateMonitor();
  move_group_interface_l->startStateMonitor();


}



//////////////////////////////////////////////////////////////////
//**************************************************************//
//////////////////////////////////////////////////////////////////

void print_info(){
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("moveit controller Right", "Planning frame: %s", move_group_interface_r->getPlanningFrame().c_str());
  
  ROS_INFO_NAMED("moveit controller Right", "Pose Reference frame: %s", move_group_interface_r->getPoseReferenceFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("moveit controller Right", "End effector link: %s", move_group_interface_r->getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("moveit controller Right", "Available Planning Groups:");
  std::copy(move_group_interface_r->getJointModelGroupNames().begin(),
            move_group_interface_r->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
            
  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("moveit controller Right", "Joint names:");
  std::copy(move_group_interface_r->getJointNames().begin(),
            move_group_interface_r->getJointNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
            
  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("moveit controller Right", "Link names:");
  std::copy(move_group_interface_r->getLinkNames().begin(),
            move_group_interface_r->getLinkNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
             
  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("moveit controller Right", "Variable Count: %d", move_group_interface_r->getVariableCount());
  
  
  ROS_INFO_NAMED("moveit controller Left", "Planning frame: %s", move_group_interface_l->getPlanningFrame().c_str());
  
  ROS_INFO_NAMED("moveit controller Left", "Pose Reference frame: %s", move_group_interface_l->getPoseReferenceFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("moveit controller Left", "End effector link: %s", move_group_interface_l->getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("moveit controller Left", "Available Planning Groups:");
  std::copy(move_group_interface_l->getJointModelGroupNames().begin(),
            move_group_interface_l->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
            
  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("moveit controller Left", "Joint names:");
  std::copy(move_group_interface_l->getJointNames().begin(),
            move_group_interface_l->getJointNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
            
  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("moveit controller Left", "Link names:");
  std::copy(move_group_interface_l->getLinkNames().begin(),
            move_group_interface_l->getLinkNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
             
  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("moveit controller Left", "Variable Count: %d", move_group_interface_l->getVariableCount());
  
  double t = move_group_interface_l->getPlanningTime();
  ROS_INFO("Current planning time: %f\n", t);


}


//////////////////////////////////////////////////////////////////
//**************************************************************//
//////////////////////////////////////////////////////////////////

void init_target_pose_orient(){
  target_pose1.orientation.x = 0.0;
  target_pose1.orientation.y = 1.0;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 0.0;
  
  target_pose2.orientation.x = 0.0;
  target_pose2.orientation.y = 1.0;
  target_pose2.orientation.z = 0.0;
  target_pose2.orientation.w = 0.0;
  
  target_pose3.orientation.x = 0.0;
  target_pose3.orientation.y = 1.0;
  target_pose3.orientation.z = 0.0;
  target_pose3.orientation.w = 0.0;
  
  target_pose4.orientation.x = 0.0;
  target_pose4.orientation.y = 1.0;
  target_pose4.orientation.z = 0.0;
  target_pose4.orientation.w = 0.0;


}

//////////////////////////////////////////////////////////////////
//**************************************************************//
//////////////////////////////////////////////////////////////////

bool plan_R(){

  success = (move_group_interface_r->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("moveit controller RIGHT", "Found plan (pose goal) %s", success ? "" : "FAILED");
  
    trajectory = my_plan.trajectory_;
    my_trajectory.arm = "right";
    my_trajectory.trajectory.push_back(trajectory);
    
    return success;

}

//////////////////////////////////////////////////////////////////
//**************************************************************//
//////////////////////////////////////////////////////////////////


bool plan_L(){

  success = (move_group_interface_l->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("moveit controller LEFT", "Found plan (pose goal) %s", success ? "" : "FAILED");
  
    trajectory = my_plan.trajectory_;
    my_trajectory.arm = "left";
    my_trajectory.trajectory.push_back(trajectory);
    
    return success;

}

//////////////////////////////////////////////////////////////////
//**************************************************************//
//////////////////////////////////////////////////////////////////

void update_joint_positions(){
   
   joint_group_positions.clear();
   int s = trajectory.joint_trajectory.points.size();
   joint_group_positions.push_back(trajectory.joint_trajectory.points[s-1].positions[0]);
   joint_group_positions.push_back(trajectory.joint_trajectory.points[s-1].positions[1]);
   joint_group_positions.push_back(trajectory.joint_trajectory.points[s-1].positions[2]);
   joint_group_positions.push_back(trajectory.joint_trajectory.points[s-1].positions[3]);
   joint_group_positions.push_back(trajectory.joint_trajectory.points[s-1].positions[4]);
   joint_group_positions.push_back(trajectory.joint_trajectory.points[s-1].positions[5]);
   joint_group_positions.push_back(trajectory.joint_trajectory.points[s-1].positions[6]);
   

}

//////////////////////////////////////////////////////////////////
//**************************************************************//
//////////////////////////////////////////////////////////////////



bool callback_pickplace(human_baxter_servers::PickPlace::Request &req, human_baxter_servers::PickPlace::Response &res){
  
  std::vector<geometry_msgs::Pose> waypoints2;
  std::vector<geometry_msgs::Pose> waypoints3;
  
  std::string hand = req.hand;
  
  target_pose1.position.x = req.pick_pose.x;
  target_pose1.position.y = req.pick_pose.y;
  target_pose1.position.z = req.pick_pose.z + 0.05;
  
  waypoints2.push_back(target_pose1);
  
  target_pose2.position = target_pose1.position;
  target_pose2.position.z -= 0.02;
  waypoints2.push_back(target_pose2);
  
  target_pose2.position.z -= 0.02;
  waypoints2.push_back(target_pose2);
  
  target_pose2.position.z -= 0.01;
  waypoints2.push_back(target_pose2);
  
  waypoints3.push_back(target_pose2);
  
  target_pose3.position = target_pose2.position;
  target_pose3.position.z += 0.02;
  waypoints3.push_back(target_pose3);
  
  target_pose3.position.z += 0.02;
  waypoints3.push_back(target_pose3);
  
  target_pose3.position.z += 0.02;
  waypoints3.push_back(target_pose3);
  
  target_pose3.position.z += 0.04;
  waypoints3.push_back(target_pose3);
  
  target_pose4.position.x = req.place_pose.x;
  target_pose4.position.y = req.place_pose.y;
  target_pose4.position.z = req.place_pose.z + 0.06;
  
  my_trajectory.trajectory.clear();
  
  //initialize variable to track trajectories planning success
  bool success_plan = true;
  

  if(hand == "right"){
    // Execute with RIGHT hand 
    
    robot_state::RobotState start_state_r(*(move_group_interface_r->getCurrentState()));
    const moveit::core::JointModelGroup* joint_model_group_r = move_group_interface_r->getCurrentState()->getJointModelGroup(move_group_interface_r->getName());
    
    ///**//
    //1st trajectory
    
    start_state_r.setJointGroupPositions(joint_model_group_r, init_baxter_state_r.joint_state.position);
    move_group_interface_r->setStartState(start_state_r);
    move_group_interface_r->setPoseTarget(target_pose1);
    
    success_plan = plan_R();
    
    if(success_plan == false){
      res.done = false;
      return true;
    }
    
    update_joint_positions();
    start_state_r.setJointGroupPositions(joint_model_group_r, joint_group_positions);
    
    
    ///**//
    //2nd trajectory
    
    move_group_interface_r->setStartState(start_state_r);
    
    move_group_interface_r->computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory);
    
    my_trajectory.arm = "right";
    my_trajectory.trajectory.push_back(trajectory);
    
    
    update_joint_positions();
    start_state_r.setJointGroupPositions(joint_model_group_r, joint_group_positions);
    
    
    ///**//
    //3rd trajectory
    
    move_group_interface_r->setStartState(start_state_r);
    
    move_group_interface_r->computeCartesianPath(waypoints3, eef_step, jump_threshold, trajectory);
    
    my_trajectory.arm = "right";
    my_trajectory.trajectory.push_back(trajectory);
    
    
    update_joint_positions();
    start_state_r.setJointGroupPositions(joint_model_group_r, joint_group_positions);
    
    
    ///**///
    //4th trajectory
    move_group_interface_r->setStartState(start_state_r);
    
    move_group_interface_r->setPoseTarget(target_pose4);
    
    success_plan = plan_R();
    if(success_plan == false){
      res.done = false;
      return true;
    }
    
  }
  else{
    // Execute with LEFT hand 
    
    robot_state::RobotState start_state_l(*(move_group_interface_l->getCurrentState()));
    const moveit::core::JointModelGroup* joint_model_group_l = move_group_interface_l->getCurrentState()->getJointModelGroup(move_group_interface_l->getName());
    
    ///**//
    //1st trajectory
    
    start_state_l.setJointGroupPositions(joint_model_group_l, init_baxter_state_l.joint_state.position);
    move_group_interface_l->setStartState(start_state_l);
    move_group_interface_l->setPoseTarget(target_pose1);
    
    success_plan = plan_L();
    if(success_plan == false){
      res.done = false;
      return true;
    }
    
    update_joint_positions();
    start_state_l.setJointGroupPositions(joint_model_group_l, joint_group_positions);
    
    
    ///**//
    //2nd trajectory
    
    move_group_interface_l->setStartState(start_state_l);
    
    
    move_group_interface_l->computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory);
    
    my_trajectory.arm = "left";
    my_trajectory.trajectory.push_back(trajectory);
    
    
    update_joint_positions();
    start_state_l.setJointGroupPositions(joint_model_group_l, joint_group_positions);
    
    
    ///**//
    //3rd trajectory
    
    move_group_interface_l->setStartState(start_state_l);
    
    
    move_group_interface_l->computeCartesianPath(waypoints3, eef_step, jump_threshold, trajectory);
    
    my_trajectory.arm = "left";
    my_trajectory.trajectory.push_back(trajectory);
    
    
    update_joint_positions();
    start_state_l.setJointGroupPositions(joint_model_group_l, joint_group_positions);
    
    
    ///**///
    //4th trajectory
    move_group_interface_l->setStartState(start_state_l);
    
    move_group_interface_l->setPoseTarget(target_pose4);
    
    success_plan = plan_L();
    if(success_plan == false){
      res.done = false;
      return true;
    }
  
  
  }
  
  // Publish the overall baxter trajectory
  baxter_trajectory_pub.publish(my_trajectory);
  res.done = true;
  
  
  return true;
}


//////////////////////////////////////////////////////////////////
//**************************************************************//
//////////////////////////////////////////////////////////////////


int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_controller_slow");
  ros::NodeHandle node_handle;

  
  baxter_trajectory_pub = node_handle.advertise<human_baxter_servers::BaxterTrajectory>("/baxter_moveit_trajectory", 10);
  
  pick_server = node_handle.advertiseService("/pickplace",callback_pickplace);
  
  
  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(4);
  spinner.start();
  //ros::spin();
  
  setup();
  init_baxter_state();
  init_move_group_param();
  //print_info();
  init_target_pose_orient();
  std::cout<<"Moveit Controller:: I called all initial functions"<<std::endl;
  
   
  ros::waitForShutdown();
  
  return 0;
}
