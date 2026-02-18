#include "ee4308_turtle/controller.hpp"

namespace ee4308::turtle
{
    void Controller::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        (void)costmap_ros;

        // initialize states / variables
        this->node_ = parent.lock(); // this class is not a node_. It is instantiated as part of a node_ `parent`.
        this->tf_ = tf;
        this->plugin_name_ = name;
        this->current_lookahead = desired_lookahead_dist_;

        // initialize parameters
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_linear_vel", this->desired_linear_vel_, 0.2);
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_lookahead_dist", this->desired_lookahead_dist_, 0.4);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_angular_vel", this->max_angular_vel_, 1.0);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_linear_vel", this->max_linear_vel_, 0.22);
        ee4308::initParam(this->node_, this->plugin_name_ + ".xy_goal_thres", this->xy_goal_thres_, 0.05);
        ee4308::initParam(this->node_, this->plugin_name_ + ".yaw_goal_thres", this->yaw_goal_thres_, 0.25);

        // initialize topics
        // this->sub_scan_ = this->node_->create_subscription<sensor_msgs::msg::LaserScan>(
        //     "scan", rclcpp::SensorDataQoS(),
        //     std::bind(&Controller::callbackSubScan_, this, std::placeholders::_1));
    }

    // void Controller::callbackSubScan_(sensor_msgs::msg::LaserScan::SharedPtr msg)
    // {
    //     this->scan_ranges_ = msg->ranges;
    // }

    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &rbt_pose_odom,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)velocity;     // not used
        (void)goal_checker; // not used

        // check if path exists
        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
            return writeCmdVel(0, 0);
        }

        // get rbt's pose in map frame (DO NOT DELETE --> need the next two lines for rbt_pose)
        geometry_msgs::msg::PoseStamped rbt_pose;
        tf_->transform(rbt_pose_odom, rbt_pose, "map");

        // get goal pose (contains the "clicked" goal rotation and position)
        geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();
        std::cout << "goal " << goal_pose.pose.position.x << " " << goal_pose.pose.position.y << std::endl;


        //  If the robot is close to the goal then stop
        double smallest_dist = 
                std::pow(rbt_pose.pose.position.x - goal_pose.pose.position.x, 2.0) 
                + std::pow(rbt_pose.pose.position.y - goal_pose.pose.position.y, 2.0);
                
        if (smallest_dist < std::pow(xy_goal_thres_, 2.0)) {
            if (std::abs(ee4308::getYawFromQuaternion(rbt_pose.pose.orientation)
                    - ee4308::getYawFromQuaternion(goal_pose.pose.orientation)) > yaw_goal_thres_) {
                return writeCmdVel(0, final_turn_omega_); // spin to goal orientation
                }
            return writeCmdVel(0, 0); // can stop as goal met
        }

        //  From the closest point, find the lookahead point.
        geometry_msgs::msg::PoseStamped lookahead_pose = goal_pose;


        //  Find the point along the path that is closest to the robot.
        for (int i = global_plan_.poses.size() - 1; i >= 0; i--) {
            geometry_msgs::msg::PoseStamped checked_pose = global_plan_.poses[i];
            std::cout << "checked " << checked_pose.pose.position.x << " " << checked_pose.pose.position.y << std::endl;
            double current_dist = 
                std::pow(rbt_pose.pose.position.x - checked_pose.pose.position.x, 2.0) 
                + std::pow(rbt_pose.pose.position.y - checked_pose.pose.position.y, 2.0);
            if (current_dist > std::pow(current_lookahead, 2.0)) {
                lookahead_pose = checked_pose;
            }
            else break;
        }

        //  Transform the lookahead point into the robot frame to get ( x ′ , y ′ ) .
        // delX = x_look - x_rbt
        double delX = lookahead_pose.pose.position.x - rbt_pose.pose.position.x;
        // delY = y_look - y_rbt
        double delY = lookahead_pose.pose.position.y - rbt_pose.pose.position.y;
        double phi = ee4308::getYawFromQuaternion(rbt_pose.pose.orientation); // getYaw returns -pi to +pi radians
        // x' = delX cos(phi) + delY sin(phi)
        double x_prime = delX * std::cos(phi) + delY * std::sin(phi);
        // y' = delY cos(phi) - delX sin(phi)
        double y_prime = delY * std::cos(phi) - delX * std::sin(phi);
        std::cout << "prime " << x_prime << " " << y_prime << std::endl;

        //  Calculate the curvature c . c = 1/r = 2y'/[(x'^2+y'^2)]
        double curv = std::min(2 * y_prime / (std::pow(x_prime, 2.0) + std::pow(y_prime, 2.0)), curvature_thres_);
        // Constrained curv in advance; curvature heuristic

        //  Calculate ω from v and c .
        //  Constrain ω to within the largest allowable angular speed.
        double linear_vel = std::min(desired_linear_vel_, max_linear_vel_);
        /*
        if (proximity) linear_vel *= d_obs / obstacle_thres_;
        */

        //  Constrain v to within the largest allowable linear speed.
        double angular_vel = (y_prime) / std::abs(y_prime) * std::min(std::abs(curv * linear_vel), max_angular_vel_);
        current_lookahead = linear_vel * lookahead_gain_; // after considering proximity heuristic
        return writeCmdVel(linear_vel, angular_vel);
    }

    geometry_msgs::msg::TwistStamped Controller::writeCmdVel(double linear_vel, double angular_vel)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = "odom";
        cmd_vel.header.stamp = this->node_->now();
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }

    // ======================================== DO NOT TOUCH =================================

    void Controller::cleanup() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::activate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::deactivate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        (void)speed_limit;
        (void)percentage;
    }

    void Controller::setPlan(const nav_msgs::msg::Path &path) { this->global_plan_ = path; }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Controller, nav2_core::Controller)