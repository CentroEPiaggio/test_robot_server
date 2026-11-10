// An additional client of the Robot Server: subscribes to Yr and Y as the
// controller does and copies them into matrices, to load the server with more
// than one consumer.

#include <memory>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <franka_server/msg/float32_array.hpp>

using FloatArray = franka_server::msg::Float32Array;

class LoadSubscriber : public rclcpp::Node {
public:
  LoadSubscriber() : Node("load_subscriber") {
    for (const auto & name : {"Yr", "Y"}) {
      subscriptions_.push_back(create_subscription<FloatArray>(
        std::string("franka_server/") + name, rclcpp::QoS(1).best_effort(),
        [this](FloatArray::SharedPtr message) {
          if (message->data.size() != 700) return;
          for (int row = 0; row < 7; ++row)
            for (int column = 0; column < 100; ++column)
              matrix_(row, column) = message->data[row * 100 + column];
          ++received_;
        }));
    }
  }
  ~LoadSubscriber() override { RCLCPP_INFO(get_logger(), "received %ld messages", received_); }

private:
  Eigen::Matrix<double, 7, 100> matrix_;
  long received_ = 0;
  std::vector<rclcpp::Subscription<FloatArray>::SharedPtr> subscriptions_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<LoadSubscriber>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
