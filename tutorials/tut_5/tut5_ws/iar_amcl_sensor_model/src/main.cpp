
// Tutorial: Writing a simple publisher and subscriber (C++) 
// https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "iar_amcl/amcl_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<iar_amcl::AmclNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
