/** include the libraries you need in your planner here */
 /** for global path planner interface */
 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>
 #include <iostream>
 #include <vector>
 #include <queue>
 #include <map>
 #include <algorithm>
 #include <cmath>

 using std::string;

 #ifndef GLOBAL_PLANNER_CPP
 #define GLOBAL_PLANNER_CPP

 namespace Astar_global_planner {
  
  class Cell
  {
  public:
      int idx;//index of cell in 1D array map[]
      double g;// path length from start
      double h;//heuristic i.e. diagpnal distance to goal
      double f;//priority
      Cell(int idx, double h, double g);
  };

 class Astar : public nav_core::BaseGlobalPlanner {
 public:

    Astar();
    Astar(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan
                );

private:

    //unsigned int posn_to_occgrid_idx(costmap_2d::Costmap2D* costmap_, double x, double y, unsigned int mx, unsigned int my);
    // int dequeue(std::vector<int>* q);
    // bool bfs(unsigned char* map, int source, int destination, std::vector<int>* pred, int nx, int ny);
    // std::vector<int> shortest_path(unsigned char* map, unsigned int source, unsigned int destination, int nx, int ny);
    //void convert_to_graph(std::vector<std::vector<int> > grid_matrix, int height, int width, std::vector<std::vector<int> >* adj, int order);

    costmap_2d::Costmap2DROS* costmap_ros_;
    double step_size_, min_dist_from_robot_;
    costmap_2d::Costmap2D* costmap_;
    base_local_planner::WorldModel* world_model_; 

    // double footprintCost(double x_i, double y_i, double theta_i);

    bool initialized_;
  };

  bool operator>(const Cell& c1, const Cell& c2);
 };
 #endif