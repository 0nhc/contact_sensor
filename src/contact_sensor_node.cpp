#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <serial_driver/serial_driver.hpp>
#include <serial_driver/serial_port.hpp>
#include <memory>
#include <algorithm>
#include <cctype>
#include <sstream>

class SerialNode : public rclcpp::Node
{
public:
    SerialNode() : Node("serial_node")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("device", "/dev/arduino_rp2040_connect");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<std::string>("frame_id", "serial_frame");
        this->declare_parameter<std::string>("topic_name", "/sensor_data");

        device_ = this->get_parameter("device").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        topic_name_ = this->get_parameter("topic_name").as_string();

        // Set up the publisher
        publisher_ = this->create_publisher<std_msgs::msg::Int32>(topic_name_, 10);

        // Initialize serial port
        configure_serial_port();

        // Create a timer to read from the serial port
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&SerialNode::read_serial_data, this));
    }

private:
    void configure_serial_port()
    {
        using namespace drivers::serial_driver;

        SerialPortConfig config(baud_rate_, FlowControl::NONE, Parity::NONE, StopBits::ONE);

        try {
            io_context_ = std::make_shared<drivers::common::IoContext>();
            serial_driver_ = std::make_unique<SerialPort>(*io_context_, device_, config);
            serial_driver_->open();
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully", device_.c_str());
        } catch (const std::exception &e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
        }
    }

    void read_serial_data()
    {
        if (!serial_driver_->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
            return;
        }

        try {
            std::vector<uint8_t> buffer(1024);
            auto bytes_read = serial_driver_->receive(buffer);

            if (bytes_read > 0) {
                for (size_t i = 0; i < bytes_read; ++i) {
                    char byte = static_cast<char>(buffer[i]);

                    // Check for newline or carriage return to indicate end of a message
                    if (byte == '\n' || byte == '\r') {
                        process_complete_message();
                    } else {
                        // Add byte to the buffer if it's a valid character
                        if (buffered_data_.size() < MAX_BUFFER_SIZE) {
                            buffered_data_ += byte;
                        } else {
                            // Clear buffer if it overflows
                            RCLCPP_WARN(this->get_logger(), "Buffer overflow, clearing data");
                            buffered_data_.clear();
                        }
                    }
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", e.what());
        }
    }

    void process_complete_message()
    {
        if (buffered_data_.empty()) {
            return;
        }

        // Trim whitespace from the message
        buffered_data_.erase(0, buffered_data_.find_first_not_of(" \t"));
        buffered_data_.erase(buffered_data_.find_last_not_of(" \t") + 1);

        // Validate numeric data
        if (std::all_of(buffered_data_.begin(), buffered_data_.end(), ::isdigit)) {
            try {
                int sensor_value = std::stoi(buffered_data_);

                // Publish the integer value
                std_msgs::msg::Int32 message;
                message.data = sensor_value;
                publisher_->publish(message);
            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse numeric data: '%s'", buffered_data_.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Non-numeric or invalid data received: '%s'", buffered_data_.c_str());
        }

        // Clear the buffer for the next message
        buffered_data_.clear();
    }

    // Parameters
    std::string device_;
    int baud_rate_;
    std::string frame_id_;
    std::string topic_name_;

    // ROS 2 components
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Serial driver
    std::shared_ptr<drivers::common::IoContext> io_context_;
    std::unique_ptr<drivers::serial_driver::SerialPort> serial_driver_;

    // Data buffering
    std::string buffered_data_;
    static constexpr size_t MAX_BUFFER_SIZE = 1024;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialNode>());
    rclcpp::shutdown();
    return 0;
}