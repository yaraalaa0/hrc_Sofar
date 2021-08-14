#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "human_baxter_servers/Position.h"
#include "human_baxter_servers/PickPlace.h"
#include <geometry_msgs/Point.h>
#include <string>
#include <map>
#include <stdlib.h>


std::map<std::string, std::string> blocks_status; // status of each block, whether it is on the left or right side of the table and placed or not


std::vector<geometry_msgs::Point> placement; // position at which each block should be put inside the bluebox


std::map<std::string, std::vector<geometry_msgs::Point>> hand_limits_at_rest; // limits (x,y,z) of the human's hands at rest position


geometry_msgs::Point r_hand_place; // current pose of human's right hand
geometry_msgs::Point l_hand_place; // current pose of human's left hand
geometry_msgs::Point bluebox_place; // position of the bluebox
geometry_msgs::Point middle_place;  //position of the middle point of the table

std::vector<double> baxter_positions_at_rest; // positions of the baxter joints at rest position
bool is_baxter_right_at_rest = true;  // flag to know whether the baxter is currently at rest or not
bool is_baxter_left_at_rest = true;  // flag to know whether the baxter is currently at rest or not

bool is_baxter_right_busy = false;
bool is_baxter_left_busy = false;

std::string baxter_right_block;
std::string baxter_left_block;



void subscriberCallback(const sensor_msgs::JointState::ConstPtr& baxter_msg)
{
  int flag_r = 1;
  int flag_l = 1;
  for(int i=1; i<8; i++){
    if(abs(baxter_msg->position[i] - baxter_positions_at_rest[i-1]) > 0.01){
      is_baxter_right_at_rest = false;
      flag_r = 0;
      //std::cout<<"baxter right is NOT at rest"<<std::endl;
      
      //return;
    }
  }
  for(int i=8; i<15; i++){
    if(abs(baxter_msg->position[i] - baxter_positions_at_rest[i-1]) > 0.01){
      is_baxter_left_at_rest = false;
      flag_l = 0;
      //std::cout<<"baxter left is NOT at rest"<<std::endl;
      
      //return;
    }
  }
  
  //std::cout<<"baxter is at rest"<<std::endl;
  if(flag_r){
    is_baxter_right_at_rest = true;
    std::cout<<"baxter right at rest"<<std::endl;
  }
  if(flag_l){
    is_baxter_left_at_rest = true;
    std::cout<<"baxter left at rest"<<std::endl;
  }
  
  return;
}



// Check whether the human's hand is currently at rest or not
bool check_hand_at_rest(std::string hand){
  
  //Check if the requested hand is the right
  if(hand == "right"){
    std::cout<<"TASK_MANAGER: checking right hand"<<std::endl;
    
    std::cout<<hand_limits_at_rest["right"][0].x<<std::endl;
    if(r_hand_place.x < hand_limits_at_rest["right"][0].x || r_hand_place.x > hand_limits_at_rest["right"][1].x){
      return false;
    }
    if(r_hand_place.y < hand_limits_at_rest["right"][0].y || r_hand_place.y > hand_limits_at_rest["right"][1].y){
      return false;
    }
    if(r_hand_place.z < hand_limits_at_rest["right"][0].z || r_hand_place.z > hand_limits_at_rest["right"][1].z){
      return false;
    }
    return true;
  }
  
  //Check if the requested hand is the left
  if(hand == "left"){
    if(l_hand_place.x < hand_limits_at_rest["left"][0].x || l_hand_place.x > hand_limits_at_rest["left"][1].x){
      return false;
    }
    if(l_hand_place.y < hand_limits_at_rest["left"][0].y || l_hand_place.y > hand_limits_at_rest["left"][1].y){
      return false;
    }
    if(l_hand_place.z < hand_limits_at_rest["left"][0].z || l_hand_place.z > hand_limits_at_rest["left"][1].z){
      return false;
    }
    
    return true;
  }
  
  return true;
}

bool check_block_at_final_place(double current_pos_x, double current_pos_y, geometry_msgs::Point final_pos){
  if(abs(current_pos_x - final_pos.x) > 0.1){
      return false;
    }
  if(abs(current_pos_y - final_pos.y) > 0.1){
      return false;
    }
  return true;


}

int main(int argc, char **argv){
  
  ros::init(argc, argv, "task_manager");
  ros::NodeHandle n;
  
  ros::Subscriber baxter_state_sub = n.subscribe("/baxter_joint_states",10, subscriberCallback);
  ros::ServiceClient client_tf = n.serviceClient<human_baxter_servers::Position>("/obj_position");
  ros::ServiceClient client_moveit = n.serviceClient<human_baxter_servers::PickPlace>("/pickplace");
  
  std::cout<<"TASK_MANAGER: I am waiting for services "<<std::endl;
  client_tf.waitForExistence();
  client_moveit.waitForExistence();
  std::cout<<"TASK_MANAGER: I found services "<<std::endl;
  
  baxter_positions_at_rest.push_back(0.52);
  baxter_positions_at_rest.push_back(-1.22);
  baxter_positions_at_rest.push_back(0.0);
  baxter_positions_at_rest.push_back(1.73);
  baxter_positions_at_rest.push_back(0.0);
  baxter_positions_at_rest.push_back(0.75);
  baxter_positions_at_rest.push_back(0.0);
  baxter_positions_at_rest.push_back(-0.52);
  baxter_positions_at_rest.push_back(-1.22);
  baxter_positions_at_rest.push_back(0.0);
  baxter_positions_at_rest.push_back(1.73);
  baxter_positions_at_rest.push_back(0.0);
  baxter_positions_at_rest.push_back(0.75);
  baxter_positions_at_rest.push_back(0.0);
  
  
  geometry_msgs::Point min_rightH_limits;
  geometry_msgs::Point max_rightH_limits;
  geometry_msgs::Point min_leftH_limits;
  geometry_msgs::Point max_leftH_limits;
  
  min_rightH_limits.x = 1.3;
  min_rightH_limits.y = 0.3;
  min_rightH_limits.z = 0.93;
  max_rightH_limits.x = 1.7;
  max_rightH_limits.y = 0.41;
  max_rightH_limits.z = 0.95;
  min_leftH_limits.x = 1.25;
  min_leftH_limits.y = -0.24;
  min_leftH_limits.z = 0.93;
  max_leftH_limits.x = 1.31;
  max_leftH_limits.y = -0.14;
  max_leftH_limits.z = 0.95;
  
  hand_limits_at_rest["right"].push_back(min_rightH_limits);
  hand_limits_at_rest["right"].push_back(max_rightH_limits);
  hand_limits_at_rest["left"].push_back(min_leftH_limits);
  hand_limits_at_rest["left"].push_back(max_leftH_limits);
  
  
  int count = 5;  // number of blocks that are currently not placed yet
  
  std::cout<<"TASK_MANAGER: finished initialization "<<std::endl;
  
  ros::Duration(3).sleep();
  human_baxter_servers::Position p;
  
  p.request.object = "Bluebox";
  client_tf.call(p);
  bluebox_place.x = p.response.x;
  bluebox_place.y = p.response.y;
  bluebox_place.z = p.response.z;
  
  std::cout<<"TASK_MANAGER: Received bluebox position"<<std::endl;
  
  
  p.request.object = "MiddlePlacementN";
  client_tf.call(p);
  middle_place.x = p.response.x;
  middle_place.y = p.response.y + 0.05;
  middle_place.z = p.response.z - 0.02;
  
  std::cout<<"TASK_MANAGER: Received middle position"<<std::endl;
  
  p.request.object = "E";
  client_tf.call(p);
  if(p.response.y < middle_place.y){
    blocks_status["E"] = "right";
  }
  else{
    blocks_status["E"] = "left";
  }
  
  p.request.object = "M";
  client_tf.call(p);
  if(p.response.y < middle_place.y){
    blocks_status["M"] = "right";
  }
  else{
    blocks_status["M"] = "left";
  }
  
  p.request.object = "C";
  client_tf.call(p);
  if(p.response.y < middle_place.y){
    blocks_status["C"] = "right";
    std::cout<<"TASK_MANAGER: Block C on RIGHT"<<std::endl;
  }
  else{
    blocks_status["C"] = "left";
  }
  
  p.request.object = "G";
  client_tf.call(p);
  if(p.response.y < middle_place.y){
    blocks_status["G"] = "right";
    std::cout<<"TASK_MANAGER: Block G on RIGHT"<<std::endl;
  }
  else{
    blocks_status["G"] = "left";
  }
  
  p.request.object = "I";
  client_tf.call(p);
  if(p.response.y < middle_place.y){
    blocks_status["I"] = "right";
    std::cout<<"TASK_MANAGER: Block I on RIGHT"<<std::endl;
  }
  else{
    blocks_status["I"] = "left";
  }
  
  
  geometry_msgs::Point place;
  place.x = bluebox_place.x - 0.08;
  place.y = bluebox_place.y + 0.03;
  place.z = bluebox_place.z;
  placement.push_back(place);
  
  place.x = bluebox_place.x;
  place.y = bluebox_place.y + 0.03;
  place.z = bluebox_place.z;
  placement.push_back(place);
  
  place.x = bluebox_place.x + 0.08;
  place.y = bluebox_place.y + 0.03;
  place.z = bluebox_place.z;
  placement.push_back(place);
  
  place.x = bluebox_place.x + 0.08;
  place.y = bluebox_place.y - 0.03;
  place.z = bluebox_place.z;
  placement.push_back(place);
  
  place.x = bluebox_place.x;
  place.y = bluebox_place.y - 0.03;
  place.z = bluebox_place.z;
  placement.push_back(place);
  
  
  std::map<std::string, std::string>::iterator it;
  human_baxter_servers::PickPlace pickplace_msg;
  //int place_ind = 0;  //the current placement index of the block being placed by the left baxter arm
  
  while(count>0){
    std::cout<<"TASK_MANAGER: Entered WHILE loop"<<std::endl;
  
    for(it = blocks_status.begin(); it != blocks_status.end(); it++){
      //std::cout<<"TASK_MANAGER: In the FOR Loop"<<std::endl;
      
      if(it->second == "right" && !is_baxter_right_busy){
        std::cout<<"baxter RIGHT FREE"<<std::endl;
        p.request.object = "hand_l";
        client_tf.call(p);
        l_hand_place.x = p.response.x;
        l_hand_place.y = p.response.y;
        l_hand_place.z = p.response.z;
        
        //std::cout<<"TASK_MANAGER: Received lefthand position"<<std::endl;
        
        if(check_hand_at_rest("left")){
          pickplace_msg.request.place_pose = middle_place;
          pickplace_msg.request.hand = "right";
          p.request.object = it->first;
          client_tf.call(p);
      
          pickplace_msg.request.pick_pose.x = p.response.x;
          pickplace_msg.request.pick_pose.y = p.response.y;
          pickplace_msg.request.pick_pose.z = p.response.z;
          
          client_moveit.call(pickplace_msg);

          if(pickplace_msg.response.done){
            
            is_baxter_right_busy = true;
            baxter_right_block = it->first;
            std::cout<<"TASK_MANAGER: Sent pickplace request RIGHT HAND block "<< it->first <<std::endl;
            ros::Duration(7).sleep();
            break;
          }
          
        }
      }
      
      if(it->second == "left" && !is_baxter_left_busy){
        std::cout<<"baxter LEFT FREE"<<std::endl;
        p.request.object = "hand_r";
        client_tf.call(p);
        r_hand_place.x = p.response.x;
        r_hand_place.y = p.response.y;
        r_hand_place.z = p.response.z;
        
        
        if(check_hand_at_rest("right")){
          pickplace_msg.request.place_pose = placement[5 - count];
          pickplace_msg.request.hand = "left";
          
          p.request.object = it->first;
          client_tf.call(p);
      
          pickplace_msg.request.pick_pose.x = p.response.x;
          pickplace_msg.request.pick_pose.y = p.response.y;
          pickplace_msg.request.pick_pose.z = p.response.z;
          
          
          client_moveit.call(pickplace_msg);
          
          if(pickplace_msg.response.done){
            
            is_baxter_left_busy = true;
            baxter_left_block = it->first;
            std::cout<<"TASK_MANAGER: Sent pickplace request LEFT HAND block "<< it->first <<std::endl;
            ros::Duration(7).sleep();
            break;
          }
          
        }
      }
    
    }
    ros::spinOnce();
    
    if(is_baxter_right_busy){
      if(is_baxter_right_at_rest){
        std::cout<<"baxter RIGHT AT REST & FREE"<<std::endl;
        blocks_status[baxter_right_block] = "left";
        is_baxter_right_busy = false;
      
      }
    }
    
    if(is_baxter_left_busy){
      if(is_baxter_left_at_rest){
        std::cout<<"baxter LEFT AT REST & FREE"<<std::endl;
        p.request.object = baxter_left_block;
        client_tf.call(p);
        
        if(check_block_at_final_place(p.response.x, p.response.y , placement[5 - count])){
          
          blocks_status[baxter_left_block] = "done";
          count--;
        
        }
        
        is_baxter_left_busy = false;
      }
    }
    
    
    
    ros::Duration(1).sleep();
    ros::spinOnce();
    
    std::cout<<"I FINISHED WHILE LOOP"<<std::endl;
    std::cout<<count<<std::endl;
  }



  return 0;
}
