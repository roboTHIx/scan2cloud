#include <iostream> 
#include <rclcpp/rclcpp.hpp> 

#include <laser_geometry/laser_geometry.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


//node class
class Scan2Cloud : public rclcpp::Node
{
public:
  Scan2Cloud() : Node("scan2cloud")
  {
    //create parameter for QOS pub/sub
    this->declare_parameter("qos_cloud_pub", std::string("sensor_data"));
    this->declare_parameter("qos_scan_sub", std::string("sensor_data"));

    auto qos_cloud_str = this->get_parameter("qos_cloud_pub").as_string();
    auto qos_scan_str = this->get_parameter("qos_scan_sub").as_string();

    auto qos_cloud = rclcpp::QoS(10); //default
    if(qos_cloud_str == "sensor_data")
    {
      qos_cloud = rclcpp::SensorDataQoS();
      RCLCPP_INFO(this->get_logger(), "QOS Profile for Cloud Pub: sensor_data");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "QOS Profile for Cloud Pub: default");
    }
    auto qos_scan = rclcpp::QoS(10); //default
    if(qos_scan_str == "sensor_data")
    {
      qos_scan = rclcpp::SensorDataQoS();
      RCLCPP_INFO(this->get_logger(), "QOS Profile for Scan Sub: sensor_data");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "QOS Profile for Scan Sub: default");
    }

    //create publisher
    _pub_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", qos_cloud);
    //create subscriber
    _sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", qos_scan, std::bind(&Scan2Cloud::sub_scan_callback, this, std::placeholders::_1));
  }

private:
  void sub_scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    _projector.projectLaser(*msg, cloud_msg);
    _pub_cloud->publish(cloud_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cloud;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _sub_scan;

  //projector
  laser_geometry::LaserProjection _projector;
};



int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Scan2Cloud>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
