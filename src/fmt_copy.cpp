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
        start_t=clock();
    
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
    finish_t=clock();
    totaltime=(double)(finish_t-start_t)/CLOCKS_PER_SEC;
    cout<<"\n此初始化地图的运行时间为"<<totaltime<<"秒！！！！！！！！！"<<endl;  
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
   if(buildRRT(choice))  // 如果返回路径，则继续
    {
        ROS_WARN("ddd");
        double finalcost=10000.0;
        plan.clear();
        std::vector<geometry_msgs::PoseStamped> temp_plan;
        temp_plan.clear();
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = costmap_ros_->getGlobalFrameID();  // 得到目标点的地址
        TreeNode* father1;
        father1=NULL;
        TreeNode* father = goal_node_;  // 父节点指向目标点
        while(father != NULL)
        {
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = father->cell.x * map_resolution_ + map_origin_x_;  // 转换为实际坐标
            pose.pose.position.y = father->cell.y * map_resolution_ + map_origin_y_;
                
            pose.pose.orientation.x = 0.0;  // 方向四元素
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            pose.pose.orientation.w = 1;
            temp_plan.push_back(pose);
            father = father->father; // 寻找father的father
      
        }
        finish_t=clock();
        for(int i =temp_plan.size() - 1; i > -1; i--)
        {
            plan.push_back(temp_plan[i]);
        }
        publishPlan(plan);
        totaltime=(double)(finish_t-start_t)/CLOCKS_PER_SEC;
        cout<<"\n此程序的运行时间为"<<totaltime<<"秒！！！！！！！！！"<<endl;

        cout<<"\n此程序的探索OPEN点数为"<<rrt_nodes_.size()<<"个！！！！！"<<endl;
        cout<<"\n此程序的探索CLOSED点数为"<<rrt_nodes_Closed.size()<<"个！！！！！！！！"<<endl;

        finalcost=calcul_cost(goal_node_);
        cout<<"\n此程序的路径质量为"<<finalcost<<"m！！！！！！！！！"<<endl;

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
 
bool RRTPlanner::buildRRT(int choice)    //FMT
{
 for(int i = 0; i < iteration_; i++)
        {
            ROS_INFO("---iteration=%d---",i); 
            if(drawSample(choice))  // Samplefree n    rrt_nodes_0=Unvisited
            { 
                    switch(choice){ 
                    case 2: 
                         rrt_nodes_0.push_back(root_1);  //将目标点goal放入Unvisited
                        if(extendstarNode(rrt_nodes_0[0])) //rrt_nodes_0=Unvisited
                        {    return true;}
                    break;                     
                }
            }
        }
    return false;

} 
//FMT star
bool RRTPlanner::extendstarNode(TreeNode* qunvisited) 
{   
    starR=600*sqrt(log(sample_num_)/(sample_num_)); // 探索半径R  
    int small=0;   // 最小值标记
    int minF=0;    //最小F值标记
    double cost_least=1000000000000.0;  
    double costFMT=1000000000000.0;
    //initoal all vector
    rrt_nodes_Closed.clear();  // rrt_nodes_Closed=Closed
    rrt_nodes_[0]->cell.x=start_x_;
    rrt_nodes_[0]->cell.y=start_y_;   // rrt_nodes_=Open
    rrt_nodes_[0]->father=NULL;       // Open值一开始是start点，父节点为空
    ROS_INFO("---rrt_nodes_[0]x=%f,rrt_nodes_[0]y=%f---",rrt_nodes_[0]->cell.x,rrt_nodes_[0]->cell.y); 
    // rrt_nodes_0=Unvisited=qunvisited     rrt_nodes_=Open    rrt_nodes_Closed=Closed
    for(int ii = 0; ii < sample_num_+10; ii++)  //n个采样点
    {
        TreeNode* node;         // 新定义一个node点
        node = new TreeNode;
        node->father=NULL;
        costFMT=1000000000000.0;
        for(int gf=0;gf<rrt_nodes_.size();gf++)   // 寻找 rrt_nodes_=Open 中的代价最小点
        {
            if(calcul_cost(rrt_nodes_[gf]) <= costFMT)
            {
                minF=gf;
                costFMT=calcul_cost(rrt_nodes_[gf]);
            }
        } 
        node->cell.x = rrt_nodes_[minF]->cell.x;  // node=Z，提取最小点的所有数据
        node->cell.y = rrt_nodes_[minF]->cell.y;  // 位置 、 父节点
        node->father=rrt_nodes_[minF]->father;
        unvi.clear();
        rrt_nodes_newopen.clear();   // 临时Open集合，循环再次利用时需清空
        if(getnear(node))   // 获取Xnear ,  Xnear = Near(Unvisited , Z , R)
        {
            //ROS_INFO("---,near.size()=%d---",near.size()); 
            for(int inar = 0;inar<near.size();inar++)   // x属于Xnear ,循环
            { 
                TreeNode* xnear;         // 提取的数叫做 xnear   提取所有数据
                xnear =new TreeNode;
                xnear->father=NULL;      // 位置 、 父节点
                xnear->cell.x=rrt_nodes_0[near[inar]]->cell.x;  //rrt_nodes_0=Unvisited=qunvisited
                xnear->cell.y=rrt_nodes_0[near[inar]]->cell.y;
                if(getnearY(xnear))      // 获取Ynear ,  Ynear = Near(Open , x , R)  
                {
                    // ROS_INFO("---,nearY.size()=%d---",nearY.size()); 
                    cost_least=1000000000000.0;  
                    for(int inrY = 0;inrY<nearY.size();inrY++)  // y属于Ynear , 循环
                    {   
                        if(calcul_cost(rrt_nodes_[nearY[inrY]],xnear)<cost_least) // c(y)+Cost(y,x) ,Open中的点到xnear 的代价, rrt_nodes_=Open 
                        {
                            small=inrY;   // 寻找 ymin
                            cost_least=calcul_cost(rrt_nodes_[nearY[inrY]],xnear);  // c(x) = c(ymin) + Cost(ymin , x)
                        }
                    }
                   // ROS_INFO("---cost_least=%f---",cost_least); 
                    if(check_if_on_obstacle(rrt_nodes_[nearY[small]],xnear))  // 检查 边（ymin ， x）是否安全
                    {
                        xnear->father=NULL;
                        xnear->father=rrt_nodes_[nearY[small]];  // ymin 作为 x 的父节点
                        rrt_nodes_newopen.push_back(xnear);      // 放入临时Open中
                        unvi.push_back(inar);  //rrt_nodes_0=Unvisited ， Unvisited \去点{x}
                    }
                    //ROS_INFO("---,rrt_nodes_newopen=%d---",rrt_nodes_newopen.size()); 
                }  
            }     
            //ROS_INFO("---,rrt_nodes_099999]=%d---",rrt_nodes_0.size()); 
            //ROS_INFO("---,unvi=%d---",unvi.size()); 
            rrt_nodes_unvisited.clear();
            int numun=0;
            for(int iun = 0;iun<unvi.size();iun++)
            {  
                rrt_nodes_unvisited.push_back(rrt_nodes_0[near[unvi[iun]]]); // 将所有unvisited要去除的点备份为rrt_nodes_unvisited
            }
            for(int op1=0;op1<rrt_nodes_unvisited.size();op1++)   // 遍历rrt_nodes_unvisited
            {
                for(int op=0;op<rrt_nodes_0.size();op++)  // 针对所有rrt_nodes_0的点
                {
                    if(rrt_nodes_unvisited[op1]->cell.x==rrt_nodes_0[op]->cell.x && rrt_nodes_unvisited[op1]->cell.y==rrt_nodes_0[op]->cell.y) // 寻找匹配点
                    {
                        numun=op;
                        rrt_nodes_0.erase(rrt_nodes_0.begin()+numun);   // 去除rrt_nodes_0的点
                    }
                    
                }
            }
            //ROS_INFO("---,rrt_nodes_0=%d---",rrt_nodes_0.size()); 
        }
        //ROS_INFO("---,rrt_nodes_newopen=%d---",rrt_nodes_newopen.size()); 
        for(int iop = 0;iop<rrt_nodes_newopen.size();iop++)
        {
            rrt_nodes_.push_back(rrt_nodes_newopen[iop]);  // rrt_nodes_=Open,OPen+Open_new
        }
        rrt_nodes_.erase(rrt_nodes_.begin()+minF);                // OPen\{Z}
        rrt_nodes_Closed.push_back(rrt_nodes_[minF]);      // Closed + {Z}
        if(rrt_nodes_.empty()) //rrt_nodes_=Open  为空，则失败
        {
            ROS_INFO("emememe!!!!");
            return false;
        }
        if(goalReached(node))  // 判断到目标点了吗
        {
            ROS_INFO("ggggggggggggg!!!!");
            goal_node_ = node;
            return true;
        }
    } 
    return false;
} 
//get all nodes in the tree near the new node(R=starR)
bool  RRTPlanner::getnear(TreeNode* &node)   // 获取Xnear ,  Xnear = Near(Unvisited , Z , R)  node = Z
{
    near.clear();    // unvisited 邻居标记
    for(int i = 0;i<rrt_nodes_0.size();i++)   // rrt_nodes_0=Unvisited
    {
        if(sqrt(pow(rrt_nodes_0[i]->cell.x-node->cell.x,2)+pow(rrt_nodes_0[i]->cell.y-node->cell.y,2))<starR) // hypot(rrt_nodes_0[i]->cell.x - node->cell.x, rrt_nodes_0[i]->cell.y - node->cell.y)
        {
            near.push_back(i);
        }
    }
    if(near.size()==0)return false;
    else return true;
}
//get all nodes in the tree near the new node(R=starR)
bool  RRTPlanner::getnearY(TreeNode* &node)  // 获取Ynear ,  Ynear = Near(Open , x , R)   node = x
{
    nearY.clear();   // Open 邻居标记
    for(int i = 0;i<rrt_nodes_.size();i++)   // rrt_nodes_=Open
    {
        if(sqrt(pow(rrt_nodes_[i]->cell.x-node->cell.x,2)+pow(rrt_nodes_[i]->cell.y-node->cell.y,2))<starR) //hypot(rrt_nodes_[i]->cell.x - node->cell.x, rrt_nodes_[i]->cell.y - node->cell.y)
        {
            nearY.push_back(i);
        }
    }
    if(nearY.size()==0)return false;
    else return true;
}
//calculate the cost from the init node to rrtnode and the cost from rrtnode to node
double  RRTPlanner::calcul_cost( TreeNode* rrtnode,TreeNode* node)  // 计算代价， rrtnode = y ，node= xnear
{                       // rrtnode = y ，node= xnear  , c(y) + Cost(y , xnear)
    double cost=0;
    TreeNode* rrt;
    rrt=rrtnode;
    cost=sqrt(pow(rrtnode->cell.x-node->cell.x,2)+pow(rrtnode->cell.y-node->cell.y,2)); //Cost(y , xnear) hypot(rrtnode->cell.x - node->cell.x, rrtnode->cell.y - node->cell.y)
    while(rrt->father!=NULL)
    {
        cost=cost+sqrt(pow(rrt->cell.x-(rrt->father)->cell.x,2)+pow(rrt->cell.y-(rrt->father)->cell.y,2));  //c(y)  hypot(rrt->cell.x - (rrt->father)->cell.x, rrt->cell.y - (rrt->father)->cell.y)
        rrt=rrt->father;
    }
    return cost;
}
//calculate the cost from the init node to rrtnode
double  RRTPlanner::calcul_cost(TreeNode* rrtnode)
{
    double cost=0;
    TreeNode* rrt = rrtnode;
    while(rrt->father!=NULL){
        cost=cost+sqrt(pow(rrt->cell.x-(rrt->father)->cell.x,2)+pow(rrt->cell.y-(rrt->father)->cell.y,2)); // hypot(rrt->cell.x - (rrt->father)->cell.x, rrt->cell.y - (rrt->father)->cell.y)
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
    return true;
}
//check if a road is on obstacle 
bool RRTPlanner::check_if_on_obstacle(TreeNode* rrtnode,TreeNode* node)  // 判断边是否可以
{
    TreeNode* test;   // 定义边上的点
    extend_check=0.0002;
    float p=0.00;
    test=NULL;
    test = new TreeNode;   
    float dist=sqrt(pow(rrtnode->cell.x-node->cell.x,2)+pow(rrtnode->cell.y-node->cell.y,2));  // 计算 |ymin , x|
    float theta=atan2(node->cell.y-rrtnode->cell.y,node->cell.x-rrtnode->cell.x);  // 角度
    int k=0;
    while(p<=dist) 
    {
    test->cell.x=rrtnode->cell.x+p*cos(theta);  // 通过 cos ，sin 得到两点之间的点
    test->cell.y=rrtnode->cell.y+p*sin(theta);
        k++;
        if(!checkIfOnObstacles(test))  // 如果点没问题，则跳过
        {
            return false;
            break;
        }
        p=p+extend_check;  // 按步长不断增加
    }
    delete test;
    return true;
}
//check if a point is on obstacle
bool RRTPlanner::checkIfOnObstacles(TreeNode* &node)  // 判断点是否可以
{
    unsigned int cost = static_cast<int>(costmap_->getCost(int(node->cell.x) , int(node->cell.y )));   // 取该点在地图中的代价          
    if (cost<=threshold_)
    {
        return true;
    }
    else{
        return false;
    }
}
//generate some random points on the map
void RRTPlanner::addRandomNode()
{
    Vertex.clear();
    Vertex.push_back(rrt_nodes_[0]);
    Vertex.push_back(rrt_nodes_1[0]);
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
        qrand->cell.QCOST = 1000000000.0; 

        if(checkIfOnObstacles(qrand))    // 判断点是否位于free空间
        {
        numrand++;
        rrt_nodes_0.push_back(qrand);
        Vertex.push_back(qrand);
        }
    }
}

//initial the map 
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
