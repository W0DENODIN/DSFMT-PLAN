/*****************************************
Created by Jiaxin Guo, 2019.4.21 from zju
*****************************************/
#ifndef _RRT_H
#define _RRT_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
#include <vector>
#include <algorithm>
#include <time.h>
#include <ctime>
#include <stdio.h>
#include <math.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <boost/foreach.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

//#include <pcl_conversions/pcl_conversions.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <set>
#include <visualization_msgs/Marker.h>		//point----1

namespace rrt_plan
{
struct PointCell
{
  float x;
  float y;
  float QCOST;
  float GVGD;
  float PRA[500][1];
};
 
struct TreeNode
{
  PointCell cell;
  TreeNode* father;
  std::vector<TreeNode*> children;
};

clock_t start_t,finish_t;
double totaltime;

class RRTPlanner : public nav_core::BaseGlobalPlanner
{
public:
  void getCorrdinate (float &x, float &y)
    {
        x = x - originX;
        y = y - originY;
    }
  int k=0;
  int mapSize;
  int choice;
  double starR;
  double GVGR;
  int smooth;
  double extend_check;
  RRTPlanner();
  ~RRTPlanner();
  bool initialized_;
  RRTPlanner (ros::NodeHandle &); //this constructor is may be not needed
  RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  sensor_msgs::PointCloud t;
  ros::NodeHandle ROSNodeHandle;
  void clean_all(std::vector<geometry_msgs::PoseStamped>& plan);
    /** overriden classes from interface nav_core::BaseGlobalPlanner **/
  void transfer_map_to_cell(PointCell tree);

private:
  bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);
  bool publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
 
  TreeNode* root_;
  TreeNode* root_1;
  TreeNode* goal_node_;
  TreeNode* goal_connect_;
  bool buildRRT(int choice);
  bool drawSample(int choice);
  bool extendSingleStep(TreeNode* rrtnode, TreeNode* &node, TreeNode* random);  
  void probSample();
  void addGoalOrientNode();
  void addRandomNode();
  bool goalReached(const TreeNode* nearestNode);
  bool GgoalReached(const TreeNode* nearestNode);
  void map_initial(costmap_2d::Costmap2DROS* costmap_ros);
  void initialPoseCB(costmap_2d::Costmap2DROS* costmap_ros);
  void goalCB(costmap_2d::Costmap2DROS* costmap_ros);
  bool use_rrt_base(int choice);
  bool use_rrt_connect(int choice);
  bool use_rrt_star(int choice);
  int getconnected(std::vector<TreeNode*> startNode,TreeNode* endnode);
  bool getnear(TreeNode* &node);
  bool GVgetnear(TreeNode* &node);
  bool getnearY(TreeNode* &node);
  bool getStartGoal(TreeNode* &node);
  double calcul_cost(TreeNode* rrtnode,TreeNode* node);
  double calcul_cost( TreeNode* rrtnode);
  bool Environ(TreeNode* Xcrrunt,TreeNode* Xparent);
  bool extendstarNode(TreeNode* qunvisited);      
  bool PRM_Edges(TreeNode* qunvisited);   
  bool PRM_AfindPath(TreeNode* qunvisited);      
  bool check_if_on_obstacle(TreeNode* rrtnode,TreeNode* node);
  bool checkIfOnObstacles(TreeNode* &node);
  void pinghua();
  double IFOT(TreeNode* fangO,TreeNode* Z,TreeNode* Xcrrunt);
  bool min_to_max(TreeNode* gdot);
  bool extendAfindPath(TreeNode* qunvisited);  
  double map_resolution_;
  int map_sizex_;
  int map_sizey_;
  double map_origin_x_, map_origin_y_;
  std::vector<int> map_info_;
  std::string frame_;
  std::map<int, PointCell> freespace_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  double step_size_, min_dist_from_robot_;
  costmap_2d::Costmap2D* costmap_;
  
  double distance_, probability_;
  double threshold_;
  double grav;
  int gravity;
  double Cost;
  std::default_random_engine gen_;
  std::random_device rd_;
  int iteration_;
  int sample_num_;
  double extend_step_;
  std::vector<TreeNode*> rrt_nodes_;
  std::vector<TreeNode*> rrt_nodes_0;
  std::vector<TreeNode*> rrt_nodes_newopen;
  std::vector<TreeNode*> rrt_nodes_unvisited;
  std::vector<TreeNode*> rrt_nodes_Closed;
  std::vector<TreeNode*> rrt_nodes_1;
  std::vector<TreeNode*> rrt_new_;
  std::vector<TreeNode*> rrt_fangO;
  std::vector<TreeNode*> Aclosed;
  std::vector<TreeNode*> AOpen;
  std::vector<TreeNode*> dotP;
  std::vector<TreeNode*> obstcle_1;
  std::vector<TreeNode*> obstcle_2;
  std::vector<TreeNode*> obstcle_3;
  std::vector<TreeNode*> obstcle_4;
  std::vector<TreeNode*> obstcle_5;
  std::vector<TreeNode*> obstcle_6;
  std::vector<TreeNode*> obstcle_7;
  std::vector<TreeNode*> everydot;
  std::vector<TreeNode*> GVGdot;
  std::vector<TreeNode*> GVGPath;
  std::vector<TreeNode*> GVG_sampledot;
  std::vector<TreeNode*> GVG_goal;
  std::vector<TreeNode*> Vertex;
  std::vector<int> near;
  std::vector<int> nearG;
  std::vector<int> nearY;
  std::vector<int> unvi;
  std::vector<double> BFthat;
  std::vector<PointCell> current_sampling_0;
  std::vector<PointCell> current_sampling_1;
  
  geometry_msgs::PoseStamped start_;
  float originX;
  float originY;
  int start_x_;
  int start_y_;
  float qrand_x_;
  float qrand_y_;
  geometry_msgs::PoseStamped goal_;
  int goal_x_;
  int goal_y_;
  std::vector<geometry_msgs::PoseStamped> plan_;
 
  /* ros publisher */
  ros::Publisher plan_pub_, tree_pub_, pub_A ;
  ros::Subscriber map_sub_, pose_sub_, goal_sub_; 
};
}
#endif
