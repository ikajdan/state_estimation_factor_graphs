#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class RobotPositionEstimator : public rclcpp::Node
{
public:
    RobotPositionEstimator() : Node("state_estimation"), key_counter_(0)
    {
        amcl_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&RobotPositionEstimator::amclPoseCallback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&RobotPositionEstimator::odometryCallback, this, std::placeholders::_1));
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gtsam_pose", 10);
    }

private:
    // ROS 2 Subscriptions and Publisher
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;

    // GTSAM graph and initial estimates
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    int key_counter_;

    // Callback for AMCL pose updates
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Convert ROS pose to GTSAM Pose2
        gtsam::Pose2 pose(msg->pose.pose.position.x, msg->pose.pose.position.y,
                          getYawFromQuaternion(msg->pose.pose.orientation));

        // Create a noise model from pose covariance
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(3) << msg->pose.covariance[0], msg->pose.covariance[7], msg->pose.covariance[35]).finished());

        // Insert the initial pose estimate or create a prior factor
        if (key_counter_ == 0 || !initial_estimate_.exists(key_counter_))
        {
            initial_estimate_.insert(key_counter_, pose);
        }
        graph_.add(gtsam::PriorFactor<gtsam::Pose2>(key_counter_, pose, noise));
        // Optimize the factor graph to estimate the pose
        optimizeGraph();
        key_counter_++;
    }

    // Callback for odometry updates
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Convert ROS pose to GTSAM Pose2
        gtsam::Pose2 pose(msg->pose.pose.position.x, msg->pose.pose.position.y,
                          getYawFromQuaternion(msg->pose.pose.orientation));

        // Add a motion model as a between factor in the graph
        if (key_counter_ > 0 && initial_estimate_.exists(key_counter_ - 1))
        {
            graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
                key_counter_ - 1, key_counter_, pose, gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 0.05, 0.05, 0.01).finished())));
            initial_estimate_.insert(key_counter_, pose);
            // Optimize the factor graph to estimate the pose
            optimizeGraph();
        }
        key_counter_++;
    }

    void optimizeGraph()
    {
        if (!graph_.empty())
        {
            gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_estimate_);
            gtsam::Values result = optimizer.optimize();
            initial_estimate_ = result;
            publishCurrentEstimate();
        }
    }

    void publishCurrentEstimate()
    {
        if (initial_estimate_.exists(key_counter_ - 1))
        {
            auto pose = initial_estimate_.at<gtsam::Pose2>(key_counter_ - 1);
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = this->get_clock()->now();
            pose_msg.header.frame_id = "map";
            pose_msg.pose.pose.position.x = pose.x();
            pose_msg.pose.pose.position.y = pose.y();
            pose_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), pose.theta()));
            pose_msg.pose.covariance[0] = 0; // Not implemented, thus set to 0
            pose_publisher_->publish(pose_msg);
        }
    }

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
    {
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotPositionEstimator>());
    rclcpp::shutdown();
    return 0;
}
