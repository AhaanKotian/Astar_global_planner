#include <pluginlib/class_list_macros.h>
#include "Astar.h"
PLUGINLIB_EXPORT_CLASS(Astar_global_planner::Astar, nav_core::BaseGlobalPlanner)
namespace Astar_global_planner
{
    
    Cell::Cell(int idx, double h, double g) : idx(idx), h(h), g(g)
    {
        f = h + g;
    }
    
    Astar::Astar() : costmap_ros_(NULL), initialized_(false)
    {

    }

    Astar::Astar(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), initialized_(false)
    {
        initialize(name, costmap_ros);
    }

    void Astar::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!initialized_)
        {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            ros::NodeHandle private_nh("~/" + name);
            private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);

            initialized_ = true;
        }
        else
        ROS_WARN("This planner has already been initialized... doing nothing");
    }

    bool operator>(const Cell& c1, const Cell& c2)
    {
        return c1.f > c2.f;
    }

    bool Astar::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        if(!initialized_)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

        plan.clear();
        costmap_ = costmap_ros_->getCostmap();
        unsigned char* map = costmap_->getCharMap();
        unsigned int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
        
        double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
        
        unsigned int mx,my;

        if(goal.header.frame_id != costmap_ros_->getGlobalFrameID())
        { double start_wx = start.pose.position.x;
        double start_wy = start.pose.position.y;
        unsigned int start_idx_oned;
        if(costmap_->worldToMap(start_wx,start_wy,mx,my))
            start_idx_oned = costmap_->getIndex(mx,my);
        
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
                costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }

        double goal_wx = goal.pose.position.x;
        double goal_wy = goal.pose.position.y;
        unsigned int goal_idx_oned;
        if(costmap_->worldToMap(goal_wx,goal_wy,mx,my))
            goal_idx_oned = costmap_->getIndex(mx,my);

        double start_wx = start.pose.position.x;
        double start_wy = start.pose.position.y;
        unsigned int start_idx_oned;
        if(costmap_->worldToMap(start_wx,start_wy,mx,my))
            start_idx_oned = costmap_->getIndex(mx,my);
        
        ROS_INFO_STREAM(" source (in shortest path) = "<<start_idx_oned);
        ROS_INFO_STREAM(" destination (in shortest path) = "<<goal_idx_oned);

        std::vector<int> path;
         
        int destination = goal_idx_oned;
        int source = start_idx_oned;

        std::vector<int> pred(nx*ny,-1);
        int crawl = destination;
        path.push_back(destination);


        // A* starts
        bool b = false;
        int noac = 8; // no of adjacent cells in 8-point connectivity metric
        int adj_idx;
        std::vector<bool> visited(nx*ny,false);
        std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell> > q;//priority queue
        double world_dist = sqrt(pow((goal_wx-start_wx),2) + pow((goal_wy-start_wy),2)); //eucledian distance from start(wx,wy) to goal(wx,wy)
        double h; // heurisitic cost i.e. estimated path length/diagonal distance to goal
        h = costmap_->cellDistance(world_dist);
        double g = 0; // Path length traversed from start cell(at source cell g=0)
        Cell c(source,h,g);
        q.push(c);
        while(q.size()!=0)
        {
            Cell curr_cell = q.top();
            q.pop();
            
            int curr = curr_cell.idx;

            visited[curr]=true;
            int zi=-1,zj=-1;
            for(int i=1;i<=noac+1;i++)//i<noac since we only check 8 adjacent cells
            {  
                if(zi==-1)
                {   
                    adj_idx = curr - nx + zj;
                    if(zj==0)
                        g = curr_cell.g + 1;
                    else
                        g = curr_cell.g + 1.4;
                }
                if(zi==0)
                {
                    adj_idx = curr + zj; 
                    if(zj==0)
                        g = curr_cell.g+ 0;
                    else
                        g = curr_cell.g + 1;
                }
                if(zi==1)
                {
                    adj_idx = curr + nx + zj;
                    if(zj==0)
                        g = curr_cell.g + 1;
                    else
                        g = curr_cell.g + 1.4;
                }
                if(map[adj_idx]<128)
                {
                    if(visited[adj_idx]==false)
                    {
                        visited[adj_idx]=true;
                        pred[adj_idx]=curr;

                        costmap_->indexToCells(adj_idx,mx,my);
                        double curr_wx,curr_wy;
                        costmap_->mapToWorld(mx,my,curr_wx,curr_wy);
                        world_dist = sqrt(pow((goal_wx-curr_wx),2) + pow((goal_wy-curr_wy),2)); //euclidian distance from curr(wx,wy) to goal(wx,wy)
                        h = costmap_->cellDistance(world_dist); //euclidian distance in terms of cell distance

                        c.idx = adj_idx;
                        c.h = h; 
                        c.g = g;
                        c.f = h + g;
                        q.push(c);

                        //ROS_INFO_STREAM(" source = "<<source<<" & curr = "<<curr<<" & adj_idx = "<<int(adj_idx/100)<<" & destination = "<<int(destination/100));
                    }
                   
                    if(adj_idx==destination)
                    {   b=true;break;}
                        
                }
                if(i%3==0)
                {    zi++; zj=-1;}
                else
                    zj++;
            }
            if(b)
                break;
        }

        crawl = adj_idx;

        if(b)
        {
            while(pred[crawl]!=-1)
            {
                ROS_INFO_STREAM("pred[crawl] = "<<pred[crawl]);
                path.push_back(pred[crawl]);
                crawl=pred[crawl];
            }
            reverse(path.begin(),path.end());
            for(int i=0;i<path.size();i++)
            {
                ROS_INFO_STREAM(" index "<<i<<" = "<<path[i]);
            }
        }
        else
        {
            path.pop_back();
        }


        ROS_INFO_STREAM(" PATH.SIZE() = "<<path.size());
        ros::Time plan_time = ros::Time::now();
        for(int i=0;i<path.size();i++)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = start.header.frame_id;
            unsigned int mx1,my1;
            costmap_->indexToCells(path[i],mx1,my1);
            double wx,wy;
            costmap_->mapToWorld(mx1,my1,wx,wy);
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
            ROS_INFO_STREAM(" x = "<<plan[i].pose.position.x<<" AND y = "<<plan[i].pose.position.y);
        }

        return true;
    }





    
    
    // unsigned int posn_to_occgrid_idx(costmap_2d::Costmap2D* costmap_, double x, double y, unsigned int mx, unsigned int my)
    // {
    //     if(costmap_->worldToMap(x,y,mx,my))
    //         return costmap_->getIndex(mx,my);
    // }


    // int dequeue(std::vector<int>* q)
    // {
    //     int j=1;
    //     int r = (*q)[0];
    //     while(j<=(*q).size()-1)
    //     {
    //         (*q)[j-1]=(*q)[j];
    //         j++;
    //     }
    //     (*q).pop_back();
        
    //     return r;

    // }

    // bool bfs(unsigned char* map, int source, int destination, std::vector<int>* pred, int nx, int ny)
    // {
    //     int noac = 8; // no of adjacent cells
    //     int adj_idx,adj_y;
    //     std::vector<bool> visited(nx*ny,false);
    //     std::vector<int> q;
    //     q.push_back(source);
    //     while(q.size()!=0)
    //     {
    //         int curr = dequeue(&q);
    //         visited[curr]=true;
    //         int zi=-1,zj=-1;
    //         for(int i=1;i<=noac+1;i++)//i<noac since we only check adjacent cells
    //         {
    //             // this part change karna hoga accordingly      
    //             if(zi==-1)
    //             {   
    //                 adj_idx = curr - nx + zj;
    //             }
    //             if(zi==0)
    //             {
    //                 adj_idx = curr + zj; 
    //             }
    //             if(zi==1)
    //             {
    //                 adj_idx = curr + nx + zj;
    //             }
    //             if(map[adj_idx]<128)
    //             {
    //                 if(visited[adj_idx]==false)
    //                 {
    //                     visited[adj_idx]=true;
    //                     (*pred)[adj_idx]=curr;
    //                     q.push_back(adj_idx);
    //                 }
                    
    //                 if(adj_idx==destination)
    //                     return true;
    //             }
    //             if(i%3==0)
    //             {
    //                 zi++; zj=-1;
    //             }
    //         }
    //     }
    //     return false;
    // }
    // std::vector<int> shortest_path(unsigned char* map, unsigned int source, unsigned int destination, int nx, int ny)
    // {
    //     // ROS_INFO_STREAM(" source (in shortest path) = "<<source);
    //     // ROS_INFO_STREAM(" destination (in shortest path) = "<<destination);
    //     // ROS_INFO_STREAM(" GAYYYYYYYY 1");
    //     std::vector<int> pred(nx*ny,-1);
    //     ROS_INFO_STREAM(" GAYYYYYYYYY 2");
    //     std::vector<int> sp;
    //     int crawl = destination;
    //     std::vector<int> path;
    //     path.push_back(destination);
    //     ROS_INFO_STREAM(" GAYYYYYYYYY 3");
    //     bool b = bfs(map,source,destination,&pred,nx,ny);
    //     ROS_INFO_STREAM(" BFS output: "<<b);
    //     //cout<<"BFS output: "<<b<<endl;
    //     if(b)
    //     {
    //         while(pred[crawl]!=-1)
    //         {
    //             path.push_back(pred[crawl]);
    //             crawl=pred[crawl];
    //         }
    //         reverse(path.begin(),path.end());
    //         for(int i=0;i<20;i++)
    //         {
    //             ROS_INFO_STREAM(" index "<<i<<" = "<<path[i]);
    //         }
    //         return path;
    //     }
    //     else
    //     {
    //         path.pop_back();
    //         return path;
    //     }

    //     //std::vector<geometry_msgs::PoseStamped>& plan;

    // }
    // void convert_to_graph(std::vector<std::vector<int> > grid_matrix, int height, int width, std::vector<std::vector<int> >* adj, int order)
    // {
    //     int noac=8;//max no of adjacent cells to each cell
    //     int zi=-1,zj=-1;
    //     for(int i=0;i<height;i++)
    //     {
    //         for(int j=0;j<width;j++)
    //         {
    //             if(grid_matrix[i][j]>128)//Nodes of occupied cells shouldnt be connected to any other node (they do not exist) 
    //                 continue;
    //             for(int k=1;k<=noac+1;k++)
    //             {
    //                 if(zi==0 && zj==0)
    //                 {
    //                     zj++;
    //                     continue;
    //                 }
                    
    //                 if(i+zi>=0 && j+zj>=0 && i+zi<grid_matrix.size() && j+zj<grid_matrix[i+zi].size())
    //                 {
    //                     if(grid_matrix[i+zi][j+zj]>128)
    //                     {
    //                         (*adj)[i*grid_matrix[i].size()+j][(i*grid_matrix[i].size()+j)+(zi*grid_matrix[i].size())+zj]=0;
    //                     }
    //                     else if(grid_matrix[i+zi][j+zj]<128)
    //                     {
    //                         (*adj)[i*grid_matrix[i].size()+j][(i*grid_matrix[i].size()+j)+(zi*grid_matrix[i].size())+zj]=1;
    //                     }
    //                 }

    //                 if(k%3==0)
    //                 {
    //                     zi++;zj=-1;
    //                 }
    //                 else
    //                     zj++;
    //             }
    //             zi=-1;zj=-1;
    //         }
    //     }
    //     ROS_INFO_STREAM(" (*adj).size() = "<<(*adj).size());
    //     ROS_INFO_STREAM(" (*adj)[0].size() = "<<(*adj)[0].size());
    //     ROS_INFO_STREAM(" (*adj)[0][0] = "<<((*adj)[0][0]));
        
    //     // //Display resultant adjacency matrix
    //     // cout<<"Resultant adjacency matrix:"<<endl;
    //     // for(int i=0;i<adj.size();i++)
    //     // {
    //     //     for(int j=0;j<adj[i].size();j++)
    //     //         cout<<adj[i][j]<<" ";
    //     //     cout<<endl;
    //     // }
    // }    
};