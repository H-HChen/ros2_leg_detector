#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

class tf_transfer_node : public rclcpp::Node
{
public:
    tf_transfer_node():Node("tf_transfer_node"){
        tf_buffer =  std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        nav_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", rclcpp::SystemDefaultsQoS());
        this->declare_parameter<int>("people_num", 0);
        this->get_parameter("people_num", people_num);
    };
    void run();

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_pub;
    geometry_msgs::msg::TransformStamped people_tf;
    geometry_msgs::msg::PoseStamped goal_pose;
    int people_num;
    void transform2pose(geometry_msgs::msg::PoseStamped target_pose, geometry_msgs::msg::TransformStamped tf);
};



