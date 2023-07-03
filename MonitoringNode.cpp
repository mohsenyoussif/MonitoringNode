#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <unordered_map>
#include <std_msgs/msg/time.hpp> // Include the header for std_msgs/Time

#define POINT_CLOUD_FILTER_TRANSFORM_TIMEOUT std::chrono::milliseconds(100)
/* Code Be - // OperateSafe() 
		   - // OperateOperational () 
*/ 
#defie OperationMode() // OperateSafe() 

using namespace std::chrono_literals;

class TimeMonitoringNode : public rclcpp::Node
{
public:
  TimeMonitoringNode() : Node("time_monitoring_node")
  {
    // Initialize the timer with a 1 second interval
    timer_ = create_wall_timer(20ms, std::bind(&TimeMonitoringNode::timerCallback, this));

    // Initialize the subscribers for each node
    // Replace "node_topic" with the actual topics of the nodes you want to monitor
    subscriber_start_point_cloud_filter_transform_ = create_subscription<std_msgs::msg::Time>(
      "start_point_cloud_filter_transform_topic", 10, std::bind(&TimeMonitoringNode::callbackStartPointCloudFilterTransform, this, std::placeholders::_1));
    subscriber_end_point_cloud_filter_transform_ = create_subscription<std_msgs::msg::Time>(
      "end_point_cloud_filter_transform_topic", 10, std::bind(&TimeMonitoringNode::callbackEndPointCloudFilterTransform, this, std::placeholders::_1));

  }

private:
  // Timer callback function
  void timerCallback()
  {
    // Calculate and print the execution time for each node
	  rclcpp::Clock::SharedPtr clock = node->get_clock();
	  auto firstElement = *(node_times_.begin());
      const std::string& node_name = firstElement.first;
      const auto& start_time = firstElement.second.first;
      rclcpp::Time running_time = clock->now(); 
	  const auto& end_time = firstElement.second.second; 
      if (start_time != nullptr && end_time == nullptr) // Start working put not finished yet 
      {
	    auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(*running_time - *start_time);
        if ( execution_time > POINT_CLOUD_FILTER_TRANSFORM_TIMEOUT ) 
		{
		  RCLCPP_INFO(get_logger(), "%s execution time: %ld ms Node doesn't Work Correctly", node_name.c_str(), execution_time.count()); 
		}
      }else if (start_time != nullptr && end_time != nullptr) // Start working and finished 
	  {
		auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(*end_time - *start_time);
		RCLCPP_INFO(get_logger(), "%s execution time: %ld ms Node Worked Correctly", node_name.c_str(), execution_time.count());
	  }
  }

  // Callback functions for each node's start and end times
  void callbackStartPointCloudFilterTransform(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["point_cloud_filter_transform_nodes"].first = msg;
  }

  void callbackEndPointCloudFilterTransform(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["point_cloud_filter_transform_nodes"].second = msg;
	
  }


  // Map to store start and end times for each node
  std::unordered_map<std::string, std::pair<std_msgs::msg::Time::SharedPtr, std_msgs::msg::Time::SharedPtr>> node_times_;

  // Timer for periodically printing the execution times
  rclcpp::TimerBase::SharedPtr timer_;

  // Subscribers for each node's start and end times
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_point_cloud_filter_transform_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_point_cloud_filter_transform_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimeMonitoringNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}