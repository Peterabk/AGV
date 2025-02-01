#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <string>
#include <vector>

#include <chrono>

#include <libserial/SerialPort.h>

using namespace std::chrono_literals;


class IMUSerialReceiver : public rclcpp::Node
{
public:
  IMUSerialReceiver() : Node("imu_serial_receiver")
  {
    declare_parameter<std::string>("port", "/dev/ttyACM0");

    port_ = get_parameter("port").as_string();

    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

    pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/out", 10);
    imu_message.header.frame_id = "base_footprint";   

    timer_ = create_wall_timer(0.01s, std::bind(&IMUSerialReceiver::timerCallback, this));

  }

  ~IMUSerialReceiver()
  {
    arduino_.Close();
  }

  void timerCallback()
  {
    if(rclcpp::ok() && arduino_.IsDataAvailable())
    {

      std::string imu_string_data;
      arduino_.ReadLine(imu_string_data);

    try
    {
        imu_message.header.stamp = this->get_clock()->now(); // Set the current time
                 // Set a valid frame ID
        // Parse the IMU data
        std::vector<std::string> data_parts = splitString(imu_string_data, ',');
        if (data_parts.size() != 6)
        {
            throw std::runtime_error("Unexpected number of IMU data parts");
        }

        imu_message.linear_acceleration.x = std::stof(data_parts[0]);
        imu_message.linear_acceleration.y = std::stof(data_parts[1]);
        imu_message.linear_acceleration.z = std::stof(data_parts[2]);
        imu_message.angular_velocity.x = std::stof(data_parts[3]);
        imu_message.angular_velocity.y = std::stof(data_parts[4]);
        imu_message.angular_velocity.z = std::stof(data_parts[5]);
        // Publish the IMU message
        pub_->publish(imu_message);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error parsing IMU data: %s", e.what());
    }
    }
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string port_;
  LibSerial::SerialPort arduino_;
  sensor_msgs::msg::Imu imu_message = sensor_msgs::msg::Imu();

    // Helper function to split a string by a delimiter
    std::vector<std::string> splitString(const std::string &str, char delimiter)
    {
        std::vector<std::string> tokens;
        std::istringstream stream(str);
        std::string token;
        while (std::getline(stream, token, delimiter))
        {
            tokens.push_back(token);
        }
        return tokens;
    }


};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMUSerialReceiver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}