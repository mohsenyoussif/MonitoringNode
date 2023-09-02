#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <unordered_map>
#include <std_msgs/msg/time.hpp> // Include the header for std_msgs/Time

#define POINT_CLOUD_FILTER_TRANSFORM_TIMEOUT std::chrono::milliseconds(100)
#define POINT_CLOUD_FUSION_TIMEOUT std::chrono::milliseconds(200)
#define RAY_GROUND_CLASSIFIER_TIMEOUT std::chrono::milliseconds(150)
#define EUCLIDEAN_CLUSTER_TIMEOUT std::chrono::milliseconds(100)
#define VOXEL_GRID_TIMEOUT std::chrono::milliseconds(100)
#define NDT1_TIMEOUT std::chrono::milliseconds(200)
#define NDT2_TIMEOUT std::chrono::milliseconds(200)
#define OBJECT_COLLISION_ESTIMATOR_TIMEOUT std::chrono::milliseconds(150)
#define BEHAVIOR_PLANNER_TIMEOUT std::chrono::milliseconds(100)
#define LANE_PLANNER_TIMEOUT std::chrono::milliseconds(100)
#define PARKING_PLANNER_TIMEOUT std::chrono::milliseconds(100)
#define LANELET2_GLOBAL_PLANNER_TIMEOUT std::chrono::milliseconds(200)
#define LANELET2_MAP_PROVIDER_TIMEOUT std::chrono::milliseconds(150)
#define MPC_CONTROLLER_TIMEOUT std::chrono::milliseconds(100)

/* Code Be - // OperateSafe() 
		   - // OperateOperational () 
*/ 
#define OperationMode() // OperateSafe() 

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

    subscriber_start_point_cloud_fusion_ = create_subscription<std_msgs::msg::Time>(
      "start_point_cloud_fusion_topic", 10, std::bind(&TimeMonitoringNode::callbackStartPointCloudFusion, this, std::placeholders::_1));
    subscriber_end_point_cloud_fusion_ = create_subscription<std_msgs::msg::Time>(
      "end_point_cloud_fusion_topic", 10, std::bind(&TimeMonitoringNode::callbackEndPointCloudFusion, this, std::placeholders::_1));

    subscriber_start_ray_ground_classifier_ = create_subscription<std_msgs::msg::Time>(
      "start_ray_ground_classifier_topic", 10, std::bind(&TimeMonitoringNode::callbackStartRayGroundClassifier, this, std::placeholders::_1));
    subscriber_end_ray_ground_classifier_ = create_subscription<std_msgs::msg::Time>(
      "end_ray_ground_classifier_topic", 10, std::bind(&TimeMonitoringNode::callbackEndRayGroundClassifier, this, std::placeholders::_1));

    subscriber_start_euclidean_cluster_ = create_subscription<std_msgs::msg::Time>(
      "start_euclidean_cluster_topic", 10, std::bind(&TimeMonitoringNode::callbackStartEuclideanCluster, this, std::placeholders::_1));
    subscriber_end_euclidean_cluster_ = create_subscription<std_msgs::msg::Time>(
      "end_euclidean_cluster_topic", 10, std::bind(&TimeMonitoringNode::callbackEndEuclideanCluster, this, std::placeholders::_1));

    subscriber_start_voxel_grid_ = create_subscription<std_msgs::msg::Time>(
      "start_voxel_grid_topic", 10, std::bind(&TimeMonitoringNode::callbackStartVoxelGrid, this, std::placeholders::_1));
    subscriber_end_voxel_grid_ = create_subscription<std_msgs::msg::Time>(
      "end_voxel_grid_topic", 10, std::bind(&TimeMonitoringNode::callbackEndVoxelGrid, this, std::placeholders::_1));

    subscriber_start_ndt1_ = create_subscription<std_msgs::msg::Time>(
      "start_ndt1_topic", 10, std::bind(&TimeMonitoringNode::callbackStartNDT1, this, std::placeholders::_1));
    subscriber_end_ndt1_ = create_subscription<std_msgs::msg::Time>(
      "end_ndt1_topic", 10, std::bind(&TimeMonitoringNode::callbackEndNDT1, this, std::placeholders::_1));

    subscriber_start_ndt2_ = create_subscription<std_msgs::msg::Time>(
      "start_ndt2_topic", 10, std::bind(&TimeMonitoringNode::callbackStartNDT2, this, std::placeholders::_1));
    subscriber_end_ndt2_ = create_subscription<std_msgs::msg::Time>(
      "end_ndt2_topic", 10, std::bind(&TimeMonitoringNode::callbackEndNDT2, this, std::placeholders::_1));

    subscriber_start_object_collision_estimator_ = create_subscription<std_msgs::msg::Time>(
      "start_object_collision_estimator_topic", 10, std::bind(&TimeMonitoringNode::callbackStartObjectCollisionEstimator, this, std::placeholders::_1));
    subscriber_end_object_collision_estimator_ = create_subscription<std_msgs::msg::Time>(
      "end_object_collision_estimator_topic", 10, std::bind(&TimeMonitoringNode::callbackEndObjectCollisionEstimator, this, std::placeholders::_1));

    subscriber_start_behavior_planner_ = create_subscription<std_msgs::msg::Time>(
      "start_behavior_planner_topic", 10, std::bind(&TimeMonitoringNode::callbackStartBehaviorPlanner, this, std::placeholders::_1));
    subscriber_end_behavior_planner_ = create_subscription<std_msgs::msg::Time>(
      "end_behavior_planner_topic", 10, std::bind(&TimeMonitoringNode::callbackEndBehaviorPlanner, this, std::placeholders::_1));

    subscriber_start_lane_planner_ = create_subscription<std_msgs::msg::Time>(
      "start_lane_planner_topic", 10, std::bind(&TimeMonitoringNode::callbackStartLanePlanner, this, std::placeholders::_1));
    subscriber_end_lane_planner_ = create_subscription<std_msgs::msg::Time>(
      "end_lane_planner_topic", 10, std::bind(&TimeMonitoringNode::callbackEndLanePlanner, this, std::placeholders::_1));

    subscriber_start_parking_planner_ = create_subscription<std_msgs::msg::Time>(
      "start_parking_planner_topic", 10, std::bind(&TimeMonitoringNode::callbackStartParkingPlanner, this, std::placeholders::_1));
    subscriber_end_parking_planner_ = create_subscription<std_msgs::msg::Time>(
      "end_parking_planner_topic", 10, std::bind(&TimeMonitoringNode::callbackEndParkingPlanner, this, std::placeholders::_1));

    subscriber_start_lanelet2_global_planner_ = create_subscription<std_msgs::msg::Time>(
      "start_lanelet2_global_planner_topic", 10, std::bind(&TimeMonitoringNode::callbackStartLanelet2GlobalPlanner, this, std::placeholders::_1));
    subscriber_end_lanelet2_global_planner_ = create_subscription<std_msgs::msg::Time>(
      "end_lanelet2_global_planner_topic", 10, std::bind(&TimeMonitoringNode::callbackEndLanelet2GlobalPlanner, this, std::placeholders::_1));

    subscriber_start_lanelet2_map_provider_ = create_subscription<std_msgs::msg::Time>(
      "start_lanelet2_map_provider_topic", 10, std::bind(&TimeMonitoringNode::callbackStartLanelet2MapProvider, this, std::placeholders::_1));
    subscriber_end_lanelet2_map_provider_ = create_subscription<std_msgs::msg::Time>(
      "end_lanelet2_map_provider_topic", 10, std::bind(&TimeMonitoringNode::callbackEndLanelet2MapProvider, this, std::placeholders::_1));

    subscriber_start_mpc_controller_ = create_subscription<std_msgs::msg::Time>(
      "start_mpc_controller_topic", 10, std::bind(&TimeMonitoringNode::callbackStartMPCController, this, std::placeholders::_1));
    subscriber_end_mpc_controller_ = create_subscription<std_msgs::msg::Time>(
      "end_mpc_controller_topic", 10, std::bind(&TimeMonitoringNode::callbackEndMPCController, this, std::placeholders::_1));
  }

private:
  // Timer callback function
  void timerCallback()
  {
    // Calculate and print the execution time for each node
	 for (const auto& pair : node_times_)
    {
      const std::string& node_name = pair.first;
      const auto& start_time = pair.second.first;
      const auto& end_time = pair.second.second;
	  rclcpp::Clock::SharedPtr clock = node->get_clock();
	  rclcpp::Time running_time = clock->now();
	  rclcpp::Time nodeTimeOut  = = std::chrono::milliseconds(0); 
	  if (node_name == "PointCloud2FilterTransformNode")
	  	nodeTimeOut = POINT_CLOUD_FILTER_TRANSFORM_TIMEOUT;
	  else if (node_name == "PointCloudFusionNode")
	  	nodeTimeOut = POINT_CLOUD_FUSION_TIMEOUT;
	  else if (node_name == "RayGroundClassifierCloudNode")
	  	nodeTimeOut = RAY_GROUND_CLASSIFIER_TIMEOUT;
	  else if (node_name == "EuclideanClusterNode")
	  	nodeTimeOut = EUCLIDEAN_CLUSTER_TIMEOUT;
	  else if (node_name == "VoxelCloudNode")
	  	nodeTimeOut = VOXEL_GRID_TIMEOUT;
	  else if (node_name == "NDTMapPublisherNode")
	  	nodeTimeOut = NDT1_TIMEOUT;
	  else if (node_name == "P2DNDTLocalizerNode")
	  	nodeTimeOut = NDT2_TIMEOUT;
	  else if (node_name == "ObjectCollisionEstimatorNode")
	  	nodeTimeOut = OBJECT_COLLISION_ESTIMATOR_TIMEOUT;
	  else if (node_name == "BehaviorPlannerNode")
	  	nodeTimeOut = BEHAVIOR_PLANNER_TIMEOUT;
	  else if (node_name == "LanePlannerNode")
	  	nodeTimeOut = LANE_PLANNER_TIMEOUT;
	  else if (node_name == "ParkingPlannerNode")
	  	nodeTimeOut = PARKING_PLANNER_TIMEOUT;
	  else if (node_name == "Lanelet2GlobalPlannerNode")
	  	nodeTimeOut = LANELET2_GLOBAL_PLANNER_TIMEOUT;
	  else if (node_name == "Lanelet2MapProviderNode")
	  	nodeTimeOut = LANELET2_MAP_PROVIDER_TIMEOUT;
	  else if (node_name == "MpcControllerNode")
	  	nodeTimeOut = MPC_CONTROLLER_TIMEOUT;
	  else
	  	nodeTimeOut = std::chrono::milliseconds(0);
	  
	  
      if (start_time != nullptr && end_time == nullptr) // Start working put not finished yet 
      {
	    auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(*running_time - *start_time);
        if ( execution_time > nodeTimeOut ) 
		{
		  RCLCPP_INFO(get_logger(), "%s execution time: %ld ms Node doesn't Work Correctly", node_name.c_str(), execution_time.count()); 
		}
      }else if (start_time != nullptr && end_time != nullptr) // Start working and finished 
	  {
		RCLCPP_INFO(get_logger(), "%s execution time: %ld ms Node Worked Correctly", node_name.c_str(), execution_time.count());
	  }
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

  void callbackStartPointCloudFusion(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["point_cloud_fusion_nodes"].first = msg;
  }

  void callbackEndPointCloudFusion(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["point_cloud_fusion_nodes"].second = msg;
  }

  void callbackStartRayGroundClassifier(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["ray_ground_classifier_nodes"].first = msg;
  }

  void callbackEndRayGroundClassifier(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["ray_ground_classifier_nodes"].second = msg;
  }

  void callbackStartEuclideanCluster(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["euclidean_cluster_nodes"].first = msg;
  }

  void callbackEndEuclideanCluster(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["euclidean_cluster_nodes"].second = msg;
  }

  void callbackStartVoxelGrid(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["voxel_grid_nodes"].first = msg;
  }

  void callbackEndVoxelGrid(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["voxel_grid_nodes"].second = msg;
  }

  void callbackStartNDT1(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["ndt1_nodes"].first = msg;
  }

  void callbackEndNDT1(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["ndt1_nodes"].second = msg;
  }

  void callbackStartNDT2(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["ndt2_nodes"].first = msg;
  }

  void callbackEndNDT2(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["ndt2_nodes"].second = msg;
  }

  void callbackStartObjectCollisionEstimator(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["object_collision_estimator_nodes"].first = msg;
  }

  void callbackEndObjectCollisionEstimator(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["object_collision_estimator_nodes"].second = msg;
  }

  void callbackStartBehaviorPlanner(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["behavior_planner_nodes"].first = msg;
  }

  void callbackEndBehaviorPlanner(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["behavior_planner_nodes"].second = msg;
  }

  void callbackStartLanePlanner(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["lane_planner_nodes"].first = msg;
  }

  void callbackEndLanePlanner(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["lane_planner_nodes"].second = msg;
  }

  void callbackStartParkingPlanner(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["parking_planner_nodes"].first = msg;
  }

  void callbackEndParkingPlanner(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["parking_planner_nodes"].second = msg;
  }

  void callbackStartLanelet2GlobalPlanner(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["lanelet2_global_planner_nodes"].first = msg;
  }

  void callbackEndLanelet2GlobalPlanner(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["lanelet2_global_planner_nodes"].second = msg;
  }

  void callbackStartLanelet2MapProvider(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["lanelet2_map_provider"].first = msg;
  }

  void callbackEndLanelet2MapProvider(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["lanelet2_map_provider"].second = msg;
  }

  void callbackStartMPCController(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["mpc_controller_nodes"].first = msg;
  }

  void callbackEndMPCController(const std_msgs::msg::Time::SharedPtr msg)
  {
    node_times_["mpc_controller_nodes"].second = msg;
  }

  // Map to store start and end times for each node
  std::unordered_map<std::string, std::pair<std_msgs::msg::Time::SharedPtr, std_msgs::msg::Time::SharedPtr>> node_times_;

  // Timer for periodically printing the execution times
  rclcpp::TimerBase::SharedPtr timer_;

  // Subscribers for each node's start and end times
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_point_cloud_filter_transform_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_point_cloud_filter_transform_;

  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_point_cloud_fusion_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_point_cloud_fusion_;

  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_ray_ground_classifier_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_ray_ground_classifier_;

  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_euclidean_cluster_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_euclidean_cluster_;

  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_voxel_grid_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_voxel_grid_;

  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_ndt1_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_ndt1_;

  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_ndt2_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_ndt2_;

  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_object_collision_estimator_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_object_collision_estimator_;

  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_behavior_planner_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_behavior_planner_;

  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_lane_planner_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_lane_planner_;

  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_parking_planner_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_parking_planner_;

  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_lanelet2_global_planner_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_lanelet2_global_planner_;

  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_lanelet2_map_provider_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_lanelet2_map_provider_;

  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_start_mpc_controller_;
  rclcpp::Subscription<std_msgs::msg::Time>::SharedPtr subscriber_end_mpc_controller_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimeMonitoringNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
