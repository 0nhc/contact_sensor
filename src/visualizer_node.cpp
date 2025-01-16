#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MarkerVisualizer : public rclcpp::Node
{
public:
    MarkerVisualizer() : Node("marker_visualizer")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("marker_topic", "/sensor_marker");
        this->declare_parameter<std::string>("sensor_topic", "/sensor_data");
        this->declare_parameter<std::string>("frame_id", "world");
        this->declare_parameter<std::string>("child_frame_id", "sensor_frame");

        marker_topic_ = this->get_parameter("marker_topic").as_string();
        sensor_topic_ = this->get_parameter("sensor_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        child_frame_id_ = this->get_parameter("child_frame_id").as_string();

        // Publisher for visualization markers
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);

        // Subscriber for sensor data
        sensor_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            sensor_topic_,
            10,
            std::bind(&MarkerVisualizer::sensor_callback, this, std::placeholders::_1));

        // Initialize static transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        broadcast_transform();

        RCLCPP_INFO(this->get_logger(), "Marker Visualizer Node Initialized");
    }

private:
    void broadcast_transform()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = frame_id_;
        transform_stamped.child_frame_id = child_frame_id_;

        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = 0.0;
        transform_stamped.transform.rotation.y = 0.0;
        transform_stamped.transform.rotation.z = 0.0;
        transform_stamped.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    void sensor_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // Create a visualization marker
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = child_frame_id_;
        marker.header.stamp = this->now();
        marker.ns = "sensor_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Ensure sensor value is within range to avoid scale issues
        float scale = static_cast<float>(msg->data);
        if (scale <= 0.0f) {
            scale = 0.1f; // Minimum scale for visualization
            // RCLCPP_WARN(this->get_logger(), "Received sensor value is zero or negative. Setting minimum scale.");
        }

        // Set position and orientation
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set scale: arrow length represents the sensor value
        marker.scale.x = scale; // Arrow length
        marker.scale.y = 0.1;   // Arrow width
        marker.scale.z = 0.1;   // Arrow height

        // Set color
        marker.color.r = 1.0f;  // Red
        marker.color.g = 0.0f;  // Green
        marker.color.b = 0.0f;  // Blue
        marker.color.a = 1.0f;  // Fully opaque

        // Publish the marker
        marker_publisher_->publish(marker);
    }

    // Parameters
    std::string marker_topic_;
    std::string sensor_topic_;
    std::string frame_id_;
    std::string child_frame_id_;

    // ROS 2 components
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_subscriber_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarkerVisualizer>());
    rclcpp::shutdown();
    return 0;
}