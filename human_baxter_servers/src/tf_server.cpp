#include "ros/ros.h"
#include "human_baxter_servers/Position.h"
#include "human_baxter_servers/UnityTf.h"
#include <geometry_msgs/Point.h>
#include <string>
#include <set>
#include <map>

std::set<std::string> objects_set;
std::map<std::string, geometry_msgs::Point> op_map;
std::string curr_object;

void subscriberCallback(const human_baxter_servers::UnityTf::ConstPtr& tf_msg)
{
  
  for(int i=0; i<tf_msg->frames.size(); i++){
    //Check if this frame is contained in the set of needed objects
    if(objects_set.find(tf_msg->frames[i].header.frame_id.c_str()) != objects_set.end()){
      op_map[tf_msg->frames[i].header.frame_id.c_str()] = tf_msg->frames[i].pose.position;
    }
    
  }
  
}

bool callback_position(human_baxter_servers::Position::Request &req, human_baxter_servers::Position::Response &res){
  
  curr_object = req.object;
  
  res.x = op_map[curr_object].x;
  res.y = op_map[curr_object].y;
  res.z = op_map[curr_object].z;
  std::cout<<"TF_SERVER: I received request "<< curr_object.c_str()<<std::endl;
  std::cout<< curr_object <<std::endl;
  return true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_server");
  ros::NodeHandle node_handle;
  
  objects_set.insert("E");
  objects_set.insert("M");
  objects_set.insert("C");
  objects_set.insert("G");
  objects_set.insert("I");
  objects_set.insert("Bluebox");
  objects_set.insert("MiddlePlacementN");
  objects_set.insert("hand_l");
  objects_set.insert("hand_r");
  
  
  ros::Subscriber tf_sub = node_handle.subscribe("/unity_tf",100, subscriberCallback);
  
  ros::ServiceServer pos_server = node_handle.advertiseService("/obj_position",callback_position);
  
  ros::spin();
  ros::shutdown();
  return 0;  
  
}
