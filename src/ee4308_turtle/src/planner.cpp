#include "ee4308_turtle/planner.hpp"

namespace ee4308::turtle
{

    // ====================== Planner Node ===================
    AStarNode::AStarNode(int new_c, int new_r) : c(new_c), r(new_r) {}

    // ======================== Nav2 Planner Plugin ===============================
    void Planner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        // initialize states / variables
        this->node_ = parent.lock(); // this class is not a node. It is instantiated as part of a node `parent`.
        this->tf_ = tf;
        this->plugin_name_ = name;
        this->costmap_ = costmap_ros->getCostmap();
        this->global_frame_id_ = costmap_ros->getGlobalFrameID();

        // declare parameters to let the node know we are using these params.
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_access_cost", this->max_access_cost_, 254);
        ee4308::initParam(this->node_, this->plugin_name_ + ".interpolation_distance", this->interpolation_distance_, 0.05);
    }

    // Converts world coordinates to cell column and cell row.
    std::pair<int, int> Planner::XYToCR_(double x, double y)
    {
        // The following functions may be used:
        //   costmap_->getOriginX()
        //   costmap_->getOriginY()
        //   costmap_->getResolution()
        //   std::ceil()
        //   std::floor()

        double dx = (x - costmap_->getResolution()/2 - costmap_->getOriginX())
                / costmap_->getResolution();
        double dy = (y - costmap_->getResolution()/2 - costmap_->getOriginY())
                / costmap_->getResolution();

        int c = std::floor(dx);
        int r = std::floor(dy);

        return {c, r};
    }
    
    // Converts cell column and cell row to world coordinates.
    std::pair<double, double> Planner::CRToXY_(int c, int r)
    {
        // X, Y world coords float
        // C, R map coords integer
        // The following functions may be used:
        //   this->costmap_->getResolution() // double in meters
        //   this->costmap_->getOriginX()
        //   this->costmap_->getOriginY()

        double x = (c + 0.5) * this->costmap_->getResolution() + this->costmap_->getOriginX();
        double y = (r + 0.5) * this->costmap_->getResolution() + this->costmap_->getOriginY();

        return {x, y};
    }


    // Converts cell column and cell row to flattened array index.
    int Planner::CRToIndex_(int c, int r)
    {
        // The following functions may be used:
        //   this->costmap_->getSizeInCellsX() // int
        //   this->costmap_->getSizeInCellsY() // int

        return r * this->costmap_->getSizeInCellsX() + c;
    }

    // Returns true if out of map, false otherwise.
    bool Planner::outOfMap_(int c, int r)
    {
        // The following functions may be used:
        //   this->costmap_->getSizeInCellsX() // int
        //   this->costmap_->getSizeInCellsY() // int
        int c_max = static_cast<int>(this->costmap_->getSizeInCellsX());
        int r_max = static_cast<int>(this->costmap_->getSizeInCellsY());
        return c < -1 || r < -1 || c >= c_max || r >= r_max;
    }

    nav_msgs::msg::Path Planner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal,
        std::function<bool()> /*cancel_checker*/)
    {
        // =========== DELETE / COMMENT LINES IN {} ONCE READY TO CODE PLANNER ===================
        /*{ // Start (for lab 1 and testing)
            nav_msgs::msg::Path path;
            path.poses.clear();
            path.header.frame_id = this->global_frame_id_;
            path.header.stamp = this->node_->now();
            
            double dx = start.pose.position.x - goal.pose.position.x;
            double dy = start.pose.position.y - goal.pose.position.y;
            int num_steps = std::floor(std::hypot(dx, dy) / 0.05);
            std::vector<AStarNode> nodes;
            for (int s = 0; s < num_steps; ++s)
            {
                geometry_msgs::msg::PoseStamped pose; 
                pose.pose.position.x = dx * s / num_steps + goal.pose.position.x;
                pose.pose.position.y =  dy * s / num_steps + goal.pose.position.y;
                path.poses.push_back(pose);
            }

            std::reverse(path.poses.begin(), path.poses.end());

            geometry_msgs::msg::PoseStamped goal_ = goal;
            goal_.header.frame_id = "";
            goal_.header.stamp = rclcpp::Time(); 
            path.poses.push_back(goal_);

            return path;
        } // End (for lab 1 and testing) */

        // =========== Initializations ===================

        // Create an open list
        OpenList<AStarNode *> open_list;

        // get the c,r map coordinates of the start and goal points
        auto [start_c, start_r] = this->XYToCR_(start.pose.position.x, start.pose.position.y);
        auto [goal_c, goal_r] = this->XYToCR_(goal.pose.position.x, goal.pose.position.y);

        // Create a vector of nodes (modify accordingly)
        std::vector<AStarNode *> nodes;
        int c_max = static_cast<int>(this->costmap_->getSizeInCellsX());
        int r_max = static_cast<int>(this->costmap_->getSizeInCellsY());
        for (int r = 0; r < r_max; ++r)
        {
            for (int c = 0; c < c_max; ++c)
            {
                AStarNode *initNode = new AStarNode(c, r);
                // h cost is euc dist, f and g INFINITY by default
                initNode->h = std::pow(std::pow(c - goal_c, 2.0) 
                        + std::pow(r - goal_r, 2.0), 0.5);
                nodes.emplace_back(initNode);
            }
        }


        // do some start node initialization (modify accordingly)
        int start_idx = this->CRToIndex_(start_c, start_r);
        AStarNode *start_node = nodes[start_idx];
        start_node->g = 0; // not moved from start
        start_node->f = start_node->g + start_node->h;
        // parent is still null
        open_list.push(start_node);

        // ================ Expansion loop ========================
        while (rclcpp::ok() && !open_list.empty())
        {
            // pop the cheapest
            AStarNode *node = open_list.top();
            open_list.pop();

            // do something if goal found
            if (goal_c == node->c && goal_r == node->r)
            {
                return this->writeToPath_(node, goal);
            }

            // do stuff in expansion loop
            if (node->expanded) continue;
            // ================ Neighbor loop ========================
            for (auto [dc, dr] : std::vector<std::pair<int, int>>{{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}})
            {
                int nb_c = node->c + dc;
                int nb_r = node->r + dr;
                int newIndex = CRToIndex_(nb_c, nb_r);
                if (nodes[newIndex] == node->parent) continue; // skip evaluation, going back to parent

                // do stuff in neighbor loop
                double newg = nodes[CRToIndex_(node->c, node->r)]->g 
                        + std::pow(dc*dc+dr*dr, 0.5) * costmap_->getCost(nb_c, nb_r); // new gcost for newnode
                if (newg < nodes[newIndex]->g) { // if lower fcost, update nodes and add to open list else ignore
                    nodes[newIndex]->g = newg;
                    nodes[newIndex]->f = nodes[newIndex]->g + nodes[newIndex]->h; // new fcost for newnode
                    nodes[newIndex]->parent = nodes[CRToIndex_(node->c, node->r)]; // update parent
                    open_list.push(nodes[newIndex]);
                }                
            }
            node->expanded = true; // update this
        }

        return this->writeToPath_(nullptr, goal); // no path
    }

    nav_msgs::msg::Path Planner::writeToPath_(
        AStarNode *goal_node,
        geometry_msgs::msg::PoseStamped goal)
    {
        // setup the path message
        nav_msgs::msg::Path path;
        path.poses.clear();
        path.header.frame_id = this->global_frame_id_;
        path.header.stamp = this->node_->now();

        // do whatever is required to get the path
        AStarNode* node = goal_node;
        while (node != nullptr)
        { 
            // convert map coordinates to world coordinates
            auto [wx, wy] = this->CRToXY_(node->c, node->r);
            
            // push the pose into the messages.
            geometry_msgs::msg::PoseStamped pose; // do not fill the header with timestamp or frame information. 
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.orientation.w = 1; // normalized quaternion
            path.poses.push_back(pose);

            // go to the next node
            node = goal_node->parent;
        }
        
        // TODO: sg quintic smoothening

        // don't forget to reverse the path!
        // reverse path.poses
        int start_ = 0;
        int end_ = path.poses.size() - 1;
        while (start_ < end_) {
            geometry_msgs::msg::PoseStamped tmp_ = path.poses[start_];
            path.poses[start_] = path.poses[end_];
            path.poses[end_] = tmp_;
            start_++;
            end_--;
        }

        // push the original goal (contains the final yaw angle of the robot)
        goal.header.frame_id = "";
        goal.header.stamp = rclcpp::Time(); // possible bug: prevents nav2 and tf2 from having time extrapolation issues.
        path.poses.push_back(goal);

        // return path;
        return path;
    }

    // ======================================== DO NOT TOUCH =================================

    void Planner::cleanup() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }

    void Planner::activate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }

    void Planner::deactivate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Planner, nav2_core::GlobalPlanner)