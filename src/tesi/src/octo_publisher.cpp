#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/octomap_msgs/msg/octomap.hpp>



using namespace std::chrono_literals;

class OctoPublisher : public rclcpp::Node
{
  public:
    OctoPublisher()
    : Node("octo_publisher")
    {
      publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_topic", 100);

      tree_ = std::make_shared<octomap::OcTree>("octomap.bt");

      // Publish after 5 seconds
      timer_ = this->create_wall_timer(5s, std::bind(&OctoPublisher::publish_octomap, this));
    }

  private:
    void publish_octomap()
    {
      octomap_msgs::msg::Octomap msg;
      octomap_msgs::binaryMapToMsg(*tree_, msg);

      RCLCPP_INFO(this->get_logger(), "Publishing octomap");

      publisher_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr publisher_;
    std::shared_ptr<octomap::OcTree> tree_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctoPublisher>());
  rclcpp::shutdown();
  return 0;
}