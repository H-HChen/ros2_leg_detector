#include "tf_transfer/tf_transfer_node.hpp"

void tf_transfer_node::transform2pose(geometry_msgs::msg::PoseStamped target_pose, geometry_msgs::msg::TransformStamped tf){
    target_pose.header = tf.header;
    target_pose.pose.position.x = tf.transform.translation.x;
    target_pose.pose.position.y = tf.transform.translation.y;
    target_pose.pose.position.z = tf.transform.translation.z;
    target_pose.pose.orientation.w = 1;
    target_pose.pose.orientation.x = 0;
    target_pose.pose.orientation.y = 0;
    target_pose.pose.orientation.z = 0;
}

void tf_transfer_node::run(){
    std::string target_frame = "people_" + std::to_string(people_num);
    try{
        people_tf = tf_buffer->lookupTransform("base_link", target_frame, tf2::TimePointZero);
    }
    catch(tf2::TransformException &ex){
        RCLCPP_ERROR(get_logger(),"%s",ex.what());
    }
    transform2pose(goal_pose, people_tf);
    nav_pub->publish(goal_pose);
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto transfer_node = std::make_shared<tf_transfer_node>() ;
    rclcpp::Rate rate(30.0);
    while (rclcpp::ok()){
        transfer_node->run();
        rclcpp::spin_some(transfer_node);
        rate.sleep();
    }
    return 0;
}