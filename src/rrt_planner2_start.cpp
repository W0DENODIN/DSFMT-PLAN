/****************************************
Created by Jiaxin Guo, 2019.4.21 from zju
*****************************************/
#include "rrt2.h"
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <sensor_msgs/PointCloud.h>
#include <dynamic_reconfigure/server.h>
#include <vector>
#include <algorithm>
#include <pluginlib/class_list_macros.h>
#include <time.h>
#include <ctime>
#include <iostream>
using namespace std;
PLUGINLIB_EXPORT_CLASS(rrt_plan::RRTPlanner, nav_core::BaseGlobalPlanner)
namespace rrt_plan
{

RRTPlanner::RRTPlanner():initialized_(false)
{

}

RRTPlanner::RRTPlanner(ros::NodeHandle &nh)
{
    ROSNodeHandle = nh;
}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}
void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
     if (!initialized_)
    {
    
    ros::NodeHandle nd("~/" + name);
    //initialize the parameters and map
    nd.getParam("iteration_", iteration_);
    nd.getParam("sample_num_", sample_num_);
    nd.getParam("extend_step_", extend_step_);
    nd.getParam("distance_", distance_);
    nd.getParam("probability_", probability_);
    nd.getParam("starR", starR);   
    nd.getParam("choice", choice);   
    nd.getParam("smooth", smooth);   
    nd.getParam("threshold_", threshold_);   
    nd.getParam("gravity", gravity);   
    nd.getParam("grav", grav); 
    nd.getParam("extend_check", extend_check);  
    map_initial(costmap_ros);
    ROS_INFO("---width=%d,height=%d---",map_sizex_,map_sizey_);        
    //pub the plan of path out 
    plan_pub_ = nd.advertise<nav_msgs::Path>("rrt_path", 1);
    initialized_ = true;
    ROS_INFO("Initialized!!!!");
}
    else{
    ROS_WARN("Fail in initializing");
}
}

RRTPlanner::~RRTPlanner()
{
    for(int i = 0; i < rrt_nodes_.size(); i++)
    {
        delete rrt_nodes_[i];
    }
    for(int i = 0; i < rrt_nodes_0.size(); i++)
    {
        delete rrt_nodes_0[i];
    }
    for(int i = 0; i < rrt_nodes_1.size(); i++)
    {
        delete rrt_nodes_1[i];
    }
    for(int i = 0; i < rrt_new_.size(); i++)
    {
        delete rrt_new_[i];
    }

}
void RRTPlanner::clean_all(std::vector<geometry_msgs::PoseStamped>& plan){
    plan.clear();
    rrt_new_.clear();
    rrt_nodes_.clear();
    rrt_nodes_0.clear();
    rrt_nodes_1.clear();
    rrt_nodes_Closed.clear();
    k=0;
    root_ = new TreeNode;
    root_->father = NULL;
    root_->children.clear();
    root_1 = new TreeNode;
    root_1->father = NULL;
    root_1->children.clear();

} 

void RRTPlanner::transfer_map_to_cell(PointCell tree){
    tree.x=(tree.x- map_origin_x_) / map_resolution_;
    tree.y=(tree.y- map_origin_y_) / map_resolution_;
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan)
{    
     if (!initialized_)
    {
        ROS_ERROR("---The planner has not been initialized, please call initialize() to use the planner---");
        return false;
    }

    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
        ROS_ERROR("---This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.---",
                  costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
        return false;
    }
    start_t=clock();
    //t1=getCurrentTime();       
    clean_all(plan);
    tf::Stamped < tf::Pose > goal_tf;
    tf::Stamped < tf::Pose > start_tf;
    //transform the coordinate
    poseStampedMsgToTF(goal, goal_tf);
    poseStampedMsgToTF(start, start_tf);
    originX = costmap_->getOriginX();
    originY = costmap_->getOriginY();
    // transform the world to the cell  
    start_x_ = (start.pose.position.x- map_origin_x_) / map_resolution_;
    start_y_ = (start.pose.position.y- map_origin_y_) / map_resolution_;
    goal_x_ = (goal.pose.position.x - map_origin_x_) / map_resolution_;
    goal_y_ = (goal.pose.position.y - map_origin_y_) / map_resolution_;
    root_->cell.x = start_x_;
    root_->cell.y = start_y_;
    root_1->cell.x = goal_x_;
    root_1->cell.y = goal_y_;
    if(goal_x_<0||goal_x_>500||goal_y_<0||goal_y_>500){
        ROS_ERROR("Goal point out of the map!!!");
    }
    unsigned int cost_goal = static_cast<int>(costmap_->getCost(int(goal_x_) , int(goal_y_)));
    if(cost_goal>=threshold_){
        ROS_ERROR("Goal in the barrier!!!");
        
    }
    //put the start root to the rrt_nodes
    rrt_nodes_.push_back(root_);
    rrt_nodes_1.push_back(root_1);
    ROS_INFO("---startX=%f,startY=%f---",rrt_nodes_[0]->cell.x,rrt_nodes_[0]->cell.y);
    ROS_INFO("---goalX=%f,goalY=%f---",rrt_nodes_1[0]->cell.x,rrt_nodes_1[0]->cell.y);
    //buildRRT: mainly construct the rrt tree
   if(buildRRT(choice))
    {
        ROS_WARN("ddd");
        plan.clear();
        std::vector<geometry_msgs::PoseStamped> temp_plan;
        temp_plan.clear();
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = costmap_ros_->getGlobalFrameID();
        TreeNode* father1;
        father1=NULL;
        TreeNode* father = goal_node_;
        while(father != NULL)
        {
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = father->cell.x * map_resolution_ + map_origin_x_;
            pose.pose.position.y = father->cell.y * map_resolution_ + map_origin_y_;
                
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            pose.pose.orientation.w = 1;
            temp_plan.push_back(pose);
            father = father->father;
      
        }
        finish_t=clock();
        for(int i =temp_plan.size() - 1; i > -1; i--)
        {
            plan.push_back(temp_plan[i]);
        }
        publishPlan(plan);
        totaltime=(double)(finish_t-start_t)/CLOCKS_PER_SEC;
        cout<<"\n此程序的运行时间为"<<totaltime<<"秒！"<<endl;
        initialized_ = true;
        return true;
    }
    else
    {
        ROS_ERROR("We can not find a plan in %d cycles.", iteration_);
        return false;
    }    
}
//pub the path out
bool RRTPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    if(!path.empty())
    {
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;
    }
    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];     
    }
    plan_pub_.publish(gui_path);
}
 
bool RRTPlanner::buildRRT(int choice)
{
     int sizunvi;
 for(int i = 0; i < iteration_; i++)
        {
            if(drawSample(choice))
            {
                        rrt_nodes_0.push_back(root_1);
                        sizunvi=rrt_nodes_0.size();
                        ROS_INFO("---unvisited=%d---",sizunvi);
                        ROS_INFO("---fffffffff=%f,fffffffff=%f---",rrt_nodes_0[0]->cell.x,rrt_nodes_0[0]->cell.y); 
                for(int j = 0; j < rrt_nodes_0.size(); j++)
                {   
                    switch(choice){
                    case 2:
                        if(extendstarNode(rrt_nodes_0[j]))    
                        {    return true;}
                    break;                     
                }
                }
            }
        }
    return false;

} 
//RRT star
bool RRTPlanner::extendstarNode(TreeNode* random)
{
      int j,small=0;
    double path_length = 1000000000000;
    double cost_least=10000000;
    //find the nearest nodes of sample points
    rrt_nodes_[0]->cell.x=start_x_;
    rrt_nodes_[0]->cell.y=start_y_; 
    for(int i = 0; i < rrt_nodes_.size(); i++)
    {
        if(hypot(rrt_nodes_[i]->cell.x - random->cell.x, rrt_nodes_[i]->cell.y - random->cell.y) <= path_length)
        {
            path_length = hypot(rrt_nodes_[i]->cell.x - random->cell.x, rrt_nodes_[i]->cell.y - random->cell.y);
           j = i;
        }
    }
    //add new node to extend from current node to the nearest sample node
    TreeNode* node;
    node = new TreeNode;
    node->father=NULL;
    if(extendSingleStep(rrt_nodes_[j], node, random))
    {   
        if(getnear(node)){
        //node_nearest to node_init plus node_nearest to node_new 
        cost_least=calcul_cost(rrt_nodes_[j],node);
        for(int i = 0;i<near.size();i++){
        if(calcul_cost(rrt_nodes_[near[i]],node)<cost_least){
            small=i;
            cost_least=calcul_cost(rrt_nodes_[near[i]],node);
        }
        }
        node->father=NULL;
        node->father=rrt_nodes_[near[small]];
        rrt_nodes_[near[small]]->children.clear();
        rrt_nodes_[near[small]]->children.push_back(node);
        rrt_nodes_.push_back(node);
        //END FIRST STEP
        near.erase(near.begin()+small);
        if(j>=5){
        for(int i = 0;i<near.size();i++){
        if(calcul_cost(rrt_nodes_[near[i]])>calcul_cost(node)+calcul_cost(node,rrt_nodes_[near[i]])){
            rrt_nodes_[near[i]]->father=NULL;
            rrt_nodes_[near[i]]->father=node;
            rrt_nodes_[near[i]]->children.clear();
            node->children.clear();
            node->children.push_back(rrt_nodes_[near[i]]);
        }
        }
        }
        
    }else{
              node->father = rrt_nodes_[j];
        rrt_nodes_[j]->children.push_back(node);
        rrt_nodes_.push_back(node);
  
        }
        if(goalReached(node))
        {
            goal_node_ = node;
            return true;
        }
    }
    return false;
} 
//get all nodes in the tree near the new node(R=starR)
bool  RRTPlanner::getnear(TreeNode* &node){
    near.clear();
    for(int i = 0;i<rrt_nodes_.size();i++){
        if(hypot(rrt_nodes_[i]->cell.x - node->cell.x, rrt_nodes_[i]->cell.y - node->cell.y)<starR){
            near.push_back(i);
        }
    }
    if(near.size()==0)return false;
    else return true;
}
//calculate the cost from the init node to rrtnode and the cost from rrtnode to node
double  RRTPlanner::calcul_cost( TreeNode* rrtnode,TreeNode* node){
    double cost=0;
    TreeNode* rrt;
    rrt=rrtnode;
    cost=hypot(rrtnode->cell.x - node->cell.x, rrtnode->cell.y - node->cell.y);
    while(rrt->father!=NULL){
        cost=cost+hypot(rrt->cell.x - (rrt->father)->cell.x, rrt->cell.y - (rrt->father)->cell.y);
        rrt=rrt->father;
    }
    return cost;
}
//calculate the cost from the init node to rrtnode
double  RRTPlanner::calcul_cost(TreeNode* rrtnode){
    double cost=0;
    TreeNode* rrt = rrtnode;
    while(rrt->father!=NULL){
        cost=cost+hypot(rrt->cell.x - (rrt->father)->cell.x, rrt->cell.y - (rrt->father)->cell.y);
        rrt=rrt->father;
    }
    return cost;
}
//detect if the goal node has been reached
bool RRTPlanner::goalReached(const TreeNode* nearestNode)
{
    if(sqrt(pow(nearestNode->cell.x - goal_x_, 2) + pow(nearestNode->cell.y - goal_y_, 2)) * map_resolution_ < distance_)
    {   return true;
    }else{
        return false;
    }
}
//extend a single step from the nearest node to the random node
bool RRTPlanner::extendSingleStep(TreeNode* rrtnode, TreeNode* &node, TreeNode* random)
{
    float sintheta, costheta;
    float sintheta_g, costheta_g;
    float theta,sintheta_t, costheta_t;
    sintheta = (random->cell.y - rrtnode->cell.y) / sqrt(pow((random->cell.x - rrtnode->cell.x), 2) + pow((random->cell.y - rrtnode->cell.y), 2));
    costheta = (random->cell.x - rrtnode->cell.x) / sqrt(pow((random->cell.x - rrtnode->cell.x), 2) + pow((random->cell.y - rrtnode->cell.y), 2));
    //use gravity points
    node->cell.x = rrtnode->cell.x + (extend_step_ / map_resolution_) * costheta;
    node->cell.y = rrtnode->cell.y + (extend_step_ / map_resolution_) * sintheta;
    //if(check_if_on_obstacle(rrtnode,node))
    if(checkIfOnObstacles(node))
      {  
        return true;
          }    
    else
        return false;
}
//generate some sample points, use probablity rrt
void RRTPlanner::probSample()
{   
        addRandomNode();
}
//mainly draw the sample points
bool RRTPlanner::drawSample(int choice)
{

    if(freespace_.size() == 0)
        return false;
    else
    {
        probSample();
    }
 //   ROS_INFO("success drawSample");
    return true;
}
//check if a road is on obstacle --------------------------------------------------------检查路径是否在障碍内
bool RRTPlanner::check_if_on_obstacle(TreeNode* rrtnode,TreeNode* node){
    TreeNode* test;
    float p=0.00;
    test=NULL;
    test = new TreeNode;   
    float dist=sqrt(pow(rrtnode->cell.x-node->cell.x,2)+pow(rrtnode->cell.y-node->cell.y,2));
    float theta=atan2(node->cell.y-rrtnode->cell.y,node->cell.x-node->cell.x);
    int k=0;
    while(p<=dist) {
    test->cell.x=rrtnode->cell.x+p*cos(theta);
    test->cell.y=rrtnode->cell.y+p*sin(theta);
        k++;
        if(!checkIfOnObstacles(test))
        {
            return false;
            break;
        }
        p=p+extend_check;
    }
    delete test;
    return true;
}
//check if a point is on obstacle---------------------------------------------------------------- 检查点是否在障碍物内
bool RRTPlanner::checkIfOnObstacles(TreeNode* &node){
    unsigned int cost = static_cast<int>(costmap_->getCost(int(node->cell.x) , int(node->cell.y )));            
    if (cost<=threshold_){
        return true;
    }
    else{
        return false;
    }
}
//generate some random points on the map  ----------------------------------------------------------随机点生成
void RRTPlanner::addRandomNode()
{
    rrt_nodes_0.clear();   // Unvisitied 集合
    int numrand=0;
    while(numrand < sample_num_)
    {
        TreeNode* qrand;
        qrand = new TreeNode;
        float x = rd_() % map_sizex_;
        float y = rd_() % map_sizey_;
        qrand->cell.x = x;
        qrand->cell.y = y; 

        if(checkIfOnObstacles(qrand))    // 判断点是否位于free空间
        {
        numrand++;
        rrt_nodes_0.push_back(qrand);
        }
    }
}

//initial the map ------------------------------------------------------------------------------初始化地图
void RRTPlanner::map_initial(costmap_2d::Costmap2DROS* costmap_ros)
{
    costmap_ros_ = costmap_ros;//initialize the cost map
    //get the costmap from costmap_ros
    costmap_ = costmap_ros_->getCostmap();
    //initialize the planner parameterss
    originX = costmap_->getOriginX();
    originY = costmap_->getOriginY();
    map_origin_x_ = originX;
    map_origin_y_ = originY;
    map_sizex_ = costmap_->getSizeInCellsX();
    map_sizey_ = costmap_->getSizeInCellsY();
    map_resolution_ = costmap_->getResolution();
    mapSize = map_sizex_*map_sizey_;
    ROS_WARN("---mapsizeX=%d,mapsizeY=%d---",map_sizex_,map_sizey_);
    ROS_WARN("---map_resolution_=%f",map_resolution_);
    freespace_.clear();
    int j = 0;
    int index;
      //initial the costmap to adjust the bizhangtiaojian
    for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
        {
            for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
            {
                unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
                   if(cost<=60)  
                   {map_info_.push_back(true);
                    }
                else
                    {
                    map_info_.push_back(false);
                    }
            }
        }
   for(int i = 0; i < map_sizex_ * map_sizey_; i++)
    {
          if(map_info_[i] == 0)
        {
            index = i + 1;
            ++j;
            
            PointCell cell;
            cell.x = index % map_sizex_;
            cell.y = index / map_sizex_;
            freespace_[j] = cell;
        }
        
    }
   ROS_WARN("freespace num:%d,mapsize%d,freespace in a map%f,",j,mapSize,j/mapSize);
 
}
}
