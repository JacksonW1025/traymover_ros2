#include <lidar_localization/lidar_localization_component.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Keep all localization callbacks serialized in the default group. The
  // separate TF group can publish a protected snapshot during a slow match.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  rclcpp::NodeOptions options;
  std::shared_ptr<PCLLocalization> pcl_l = std::make_shared<PCLLocalization>(options);

  executor.add_node(pcl_l->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
