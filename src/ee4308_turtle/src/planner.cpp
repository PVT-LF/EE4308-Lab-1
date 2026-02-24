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
        ee4308::initParam(this->node_, this->plugin_name_ + ".sg_half_window", this->sg_half_window_, 4);
        ee4308::initParam(this->node_, this->plugin_name_ + ".sg_order", this->sg_order_, 3);
    }

    // Converts world coordinates to cell column and cell row.
    std::pair<int, int> Planner::XYToCR_(double x, double y)
    {
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
        double x = (c + 0.5) * this->costmap_->getResolution() + this->costmap_->getOriginX();
        double y = (r + 0.5) * this->costmap_->getResolution() + this->costmap_->getOriginY();
        return {x, y};
    }


    // Converts cell column and cell row to flattened array index.
    int Planner::CRToIndex_(int c, int r)
    {
        return r * this->costmap_->getSizeInCellsX() + c;
    }

    // Returns true if out of map, false otherwise.
    bool Planner::outOfMap_(int c, int r)
    {
        return c < 0 || r < 0
                || c >= (int)this->costmap_->getSizeInCellsX() 
                || r >= (int)this->costmap_->getSizeInCellsY();
    }

    void Planner::savitzkyGolay_(nav_msgs::msg::Path &path)
    {
        int n = (int)path.poses.size();
        int m = sg_half_window_;
        int p = sg_order_;
        int w = 2 * m + 1;

        if (n < w) return; // path too short for smoothing

        // Vandermonde matrix
        Eigen::MatrixXd J(w, p + 1);
        for (int i = 0; i < w; ++i)
            for (int j = 0; j <= p; ++j)
                J(i, j) = std::pow(-m + i, j);

        
        Eigen::MatrixXd A = (J.transpose() * J).inverse() * J.transpose();
        Eigen::VectorXd kernel = A.row(0);

        // smooth x and y
        std::vector<double> xs(n), ys(n);
        for (int i = 0; i < n; ++i) {
            xs[i] = path.poses[i].pose.position.x;
            ys[i] = path.poses[i].pose.position.y;
        }

        // only smooth the middle points
        for (int i = m; i < n - m; ++i) {
            double sx = 0, sy = 0;
            for (int k = 0; k < w; ++k) {
                sx += kernel[k] * xs[i + k - m];
                sy += kernel[k] * ys[i + k - m];
            }
            path.poses[i].pose.position.x = sx;
            path.poses[i].pose.position.y = sy;
        }
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

        // Create a vector of nodes (modify accordingly)
        std::vector<AStarNode> nodes;
        for (int r = 0; r < (int)this->costmap_->getSizeInCellsY(); ++r) {
            for (int c = 0; c < (int)this->costmap_->getSizeInCellsX(); ++c) {
                nodes.emplace_back(c, r);
            }
        }

        // Create an open list
        OpenList<AStarNode *> open_list;

        // get the c,r map coordinates of the start and goal points
        auto [start_c, start_r] = this->XYToCR_(start.pose.position.x, start.pose.position.y);
        auto [goal_c, goal_r] = this->XYToCR_(goal.pose.position.x, goal.pose.position.y);

        // start node initialization
        int start_idx = this->CRToIndex_(start_c, start_r);
        AStarNode *start_node = &nodes[start_idx];
        start_node->g = 0;
        // h cost is euc dist
        start_node->h = std::hypot(start_c - goal_c, start_r - goal_r);
        start_node->f = start_node->g + start_node->h;
        open_list.push(start_node);

        // ================ Expansion loop ========================
        while (rclcpp::ok() && !open_list.empty()) {
            // pop the cheapest
            AStarNode *node = open_list.top();
            open_list.pop();

            // goal found
            if (goal_c == node->c && goal_r == node->r) {
                auto path = this->writeToPath_(node, goal);
                savitzkyGolay_(path);
                return path;
            }

            if (node->expanded) continue;
            node->expanded = true;

            // ================ Neighbor loop ========================
            for (auto [dc, dr] : 
                    std::vector<std::pair<int, int>>{{1, 0}, {1, 1}, {0, 1}, {-1, 1}, 
                    {-1, 0}, {-1, -1}, {0, -1}, {1, -1}}) {
                int nb_c = node->c + dc;
                int nb_r = node->r + dr;

                // can expand out of map
                if (outOfMap_(nb_c, nb_r)) continue;

                int nb_idx = this->CRToIndex_(nb_c, nb_r);
                AStarNode *nb_node = &nodes[nb_idx];

                if (nb_node->expanded) continue;

                // compute the cost to the neighbor
                int cell_cost = this->costmap_->getCost(nb_c, nb_r) + 1; // costmap_ contains 0 costs!!
                // std::cout << "cell cost " << cell_cost << std::endl;
                if (cell_cost - 1 > this->max_access_cost_) continue; // hard obstacle

                // double newg = node->g + std::hypot(dc, dr) * (1 + cell_cost / this->max_access_cost_);
                double newg = node->g + std::hypot(dc, dr) * cell_cost;

                if (nb_node->g > newg) {
                    nb_node->g = newg; // update costs
                    nb_node->h = std::hypot(nb_c - goal_c, nb_r - goal_r);
                    nb_node->f = nb_node->g + nb_node->h;
                    nb_node->parent = node;
                    open_list.push(nb_node); // push to openlist
                }
            }
        }

        return this->writeToPath_(nullptr, goal); // no path
    }

    nav_msgs::msg::Path Planner::writeToPath_(
        AStarNode *goal_node,
        geometry_msgs::msg::PoseStamped goal) {
        // setup the path message
        nav_msgs::msg::Path path;
        path.poses.clear();
        path.header.frame_id = this->global_frame_id_;
        path.header.stamp = this->node_->now();

        // do whatever is required to get the path
        AStarNode* node = goal_node;
        while (node != nullptr) { 
            // convert map coordinates to world coordinates
            auto [wx, wy] = this->CRToXY_(node->c, node->r);
            
            // push the pose into the messages.
            geometry_msgs::msg::PoseStamped pose; // do not fill the header with timestamp or frame information. 
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.orientation.w = 1; // normalized quaternion
            path.poses.push_back(pose);

            // go to the next node
            node = node->parent;
        }
        
        // don't forget to reverse the path!
        // reverse path.poses
        std::reverse(path.poses.begin(), path.poses.end());
        /*
        int start_ = 0;
        int end_ = (int) path.poses.size() - 1;
        while (start_ < end_) {
            geometry_msgs::msg::PoseStamped tmp_ = path.poses[start_];
            path.poses[start_] = path.poses[end_];
            path.poses[end_] = tmp_;
            start_++;
            end_--;
        } */

        // push the original goal (contains the final yaw angle of the robot)
        goal.header.frame_id = "";
        goal.header.stamp = rclcpp::Time(); // possible bug: prevents nav2 and tf2 from having time extrapolation issues.
        path.poses.push_back(goal);
        // return path;

        // Done before S-G
        // start with 1st pointer to start
        int end_ = (int) path.poses.size();
        int cur_ = 0;
        int eval_ = 0;
        bool blocked = false;
        double divs;
        // std::cout << "anyangle size " << end_ << std::endl;
        while (cur_ < end_ - 1) {
            blocked = false;
            eval_ = cur_;
            // inner loop: iterate down until LOS blocked
            while (eval_ < end_ - 1) {  
                eval_++;
                // std::cout << "anyangle cur/eval " << cur_ << "   " << eval_ << std::endl;
                // find what cells any angle line passes through cur_ -> eval_
                divs = (double) eval_ - cur_;
                // std::cout << "testing for blocking " << eval_ << "  to  " << cur_ << std::endl;
                for (int t = cur_ + 1; t < eval_; t++) { // check points
                    double check_x = path.poses[cur_].pose.position.x + (t - cur_) / divs 
                            * (path.poses[eval_].pose.position.x - path.poses[cur_].pose.position.x);
                    double check_y = path.poses[cur_].pose.position.y + (t - cur_) / divs 
                            * (path.poses[eval_].pose.position.y - path.poses[cur_].pose.position.y);
                    auto [check_c, check_r] = this->XYToCR_(check_x, check_y);
                    int cell_cost = this->costmap_->getCost(check_c, check_r);
                    // std::cout << "cost debug anyangle " << cell_cost << std::endl;
                    if (cell_cost > this->max_access_cost_ / 3) {
                        blocked = true;
                        eval_--; // eval is end point of direct angle path, not blocked point
                        break;
                    }
                }
                // blocked_check for any cell cost greater than max allowable
                if (blocked) { // test for obstacle blocking
                    // std::cout << "blocking " << eval_ << "  to  " << cur_ << std::endl;
                    // replace all middle points
                    divs = (double) eval_ - cur_;
                    for (int t = cur_ + 1; t < eval_; t++) {
                        path.poses[t].pose.position.x = path.poses[cur_].pose.position.x + (t - cur_) / divs 
                                * (path.poses[eval_].pose.position.x - path.poses[cur_].pose.position.x);
                        path.poses[t].pose.position.y = path.poses[cur_].pose.position.y + (t - cur_) / divs 
                                * (path.poses[eval_].pose.position.y - path.poses[cur_].pose.position.y);
                        // std::cout << "newx, y " << t << "  att, " << path.poses[t].pose.position.x << "  " << path.poses[t].pose.position.y << std::endl;
                    }
                    cur_ = eval_ + 1; // new start is end of last point + 1 as known blocked
                } // else no block, continue on with longer test
            }
            if (eval_ == end_ - 1) { // no more checking, end reached
                if (!blocked) { // for case where straight path possible cur -> eval, last replacement
                    // replace all points between cur_ to goal
                    // divs = (double) end_ - 1 - cur_;
                    // for (int t = cur_ + 1; t < end_ - 1; t++) { // replace points
                    //     path.poses[t].pose.position.x = (t - cur_) / divs 
                    //             * (path.poses[eval_-1].pose.position.x - path.poses[cur_].pose.position.x);
                    //     path.poses[t].pose.position.y = (t - cur_) / divs 
                    //             * (path.poses[eval_-1].pose.position.y - path.poses[cur_].pose.position.y);
                    //     std::cout << "newx, y " << t << "  att, " << path.poses[t].pose.position.x << "  " << path.poses[t].pose.position.y << std::endl;
                    // }
                }
                break; // finished, or it will be cur_ = eval_ = end-1 which breaks outer while
            }
        }
        // std::cout << "og path size " << end_ << " actual " << (int) path.poses.size() << std::endl;
        return path;
    }

    // ======================================== DO NOT TOUCH =================================

    void Planner::cleanup() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }

    void Planner::activate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }

    void Planner::deactivate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Planner, nav2_core::GlobalPlanner)