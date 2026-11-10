// The generated franka_server node, unchanged, plus what the benchmarks need:
//   * Yr and Y also exposed as services (what `services: [Yr, Y]` generates);
//   * the q parameter also published as a topic (what `topics: [q]` generates),
//     after the regressors, so a client can match a reply to its request;
//   * the duration of every timer cycle, written to a CSV on shutdown.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <pthread.h>
#include <sched.h>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <franka_server/msg/float32_array.hpp>
#include <franka_server/srv/get_value.hpp>
#include <franka_server/srv/set_value.hpp>
#include <thunder_franka/thunder_franka.h>

namespace {
using FloatArray = franka_server::msg::Float32Array;
using SteadyClock = std::chrono::steady_clock;
template<typename Matrix> std::vector<float> flatten_row_major(const Matrix& matrix) {
  std::vector<float> values; values.reserve(matrix.rows() * matrix.cols());
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) for (Eigen::Index column = 0; column < matrix.cols(); ++column) values.push_back(static_cast<float>(matrix(row, column)));
  return values;
}
double since_us(SteadyClock::time_point origin, SteadyClock::time_point instant) {
  return std::chrono::duration<double, std::micro>(instant - origin).count();
}
}  // namespace

class frankaServer : public rclcpp::Node {
public:
  frankaServer() : Node("franka_server") {
    output_ = declare_parameter<std::string>("output", "server_cycles.csv");
    cycle_start_us_.reserve(1 << 20); cycle_duration_us_.reserve(1 << 20);
    origin_ = SteadyClock::now();

    ddq_subscription_ = create_subscription<FloatArray>("/controller/ddq", rclcpp::QoS(1), [this](FloatArray::SharedPtr message) { ddq_buffer_.writeFromNonRT(message->data); });
    ddqr_subscription_ = create_subscription<FloatArray>("/controller/ddqr", rclcpp::QoS(1), [this](FloatArray::SharedPtr message) { ddqr_buffer_.writeFromNonRT(message->data); });
    dq_subscription_ = create_subscription<FloatArray>("/controller/dq", rclcpp::QoS(1), [this](FloatArray::SharedPtr message) { dq_buffer_.writeFromNonRT(message->data); });
    dqr_subscription_ = create_subscription<FloatArray>("/controller/dqr", rclcpp::QoS(1), [this](FloatArray::SharedPtr message) { dqr_buffer_.writeFromNonRT(message->data); });
    par_DYN_subscription_ = create_subscription<FloatArray>("/controller/par_DYN", rclcpp::QoS(1), [this](FloatArray::SharedPtr message) { par_DYN_buffer_.writeFromNonRT(message->data); });
    par_REG_subscription_ = create_subscription<FloatArray>("/controller/par_REG", rclcpp::QoS(1), [this](FloatArray::SharedPtr message) { par_REG_buffer_.writeFromNonRT(message->data); });
    q_subscription_ = create_subscription<FloatArray>("/controller/q", rclcpp::QoS(1), [this](FloatArray::SharedPtr message) { q_buffer_.writeFromNonRT(message->data); });
    get_par_REG_service_ = create_service<franka_server::srv::GetValue>("franka_server/get_par_REG", [this](const std::shared_ptr<franka_server::srv::GetValue::Request>, std::shared_ptr<franka_server::srv::GetValue::Response> response) { const auto value = robot_.get_par_REG(); response->value.assign(value.data(), value.data() + 100); });
    // Added for the benchmark, same template as the generated function services.
    get_Yr_service_ = create_service<franka_server::srv::GetValue>("franka_server/get_Yr", [this](const std::shared_ptr<franka_server::srv::GetValue::Request> request, std::shared_ptr<franka_server::srv::GetValue::Response> response) {
      if (request->input.size() != 0) { RCLCPP_WARN(get_logger(), "get_Yr expects 0 explicit input values"); return; }
      const auto result = robot_.get_Yr(); response->value = flatten_row_major(result);
    });
    get_Y_service_ = create_service<franka_server::srv::GetValue>("franka_server/get_Y", [this](const std::shared_ptr<franka_server::srv::GetValue::Request> request, std::shared_ptr<franka_server::srv::GetValue::Response> response) {
      if (request->input.size() != 0) { RCLCPP_WARN(get_logger(), "get_Y expects 0 explicit input values"); return; }
      const auto result = robot_.get_Y(); response->value = flatten_row_major(result);
    });
    Yr_publisher_ = create_publisher<FloatArray>("franka_server/Yr", rclcpp::QoS(1));
    Y_publisher_ = create_publisher<FloatArray>("franka_server/Y", rclcpp::QoS(1));
    reg_G_publisher_ = create_publisher<FloatArray>("franka_server/reg_G", rclcpp::QoS(1));
    reg2dyn_publisher_ = create_publisher<FloatArray>("franka_server/reg2dyn", rclcpp::QoS(1));
    q_publisher_ = create_publisher<FloatArray>("franka_server/q", rclcpp::QoS(1));
    timer_ = create_wall_timer(std::chrono::microseconds(1000), std::bind(&frankaServer::on_timer, this));
  }

  ~frankaServer() override {
    std::ofstream file(output_);
    file << "start_us,duration_us\n";
    for (std::size_t index = 0; index < cycle_duration_us_.size(); ++index) file << cycle_start_us_[index] << "," << cycle_duration_us_[index] << "\n";
  }

private:
  void on_timer() {
    const auto start = SteadyClock::now();
    if (const auto* latest = ddq_buffer_.readFromRT(); latest && latest->size() == 7) {
      Eigen::Matrix<double, 7, 1> value; for (std::size_t index = 0; index < 7; ++index) value(index) = (*latest)[index]; robot_.set_ddq(value);
    }
    if (const auto* latest = ddqr_buffer_.readFromRT(); latest && latest->size() == 7) {
      Eigen::Matrix<double, 7, 1> value; for (std::size_t index = 0; index < 7; ++index) value(index) = (*latest)[index]; robot_.set_ddqr(value);
    }
    if (const auto* latest = dq_buffer_.readFromRT(); latest && latest->size() == 7) {
      Eigen::Matrix<double, 7, 1> value; for (std::size_t index = 0; index < 7; ++index) value(index) = (*latest)[index]; robot_.set_dq(value);
    }
    if (const auto* latest = dqr_buffer_.readFromRT(); latest && latest->size() == 7) {
      Eigen::Matrix<double, 7, 1> value; for (std::size_t index = 0; index < 7; ++index) value(index) = (*latest)[index]; robot_.set_dqr(value);
    }
    if (const auto* latest = par_DYN_buffer_.readFromRT(); latest && latest->size() == 80) {
      Eigen::Matrix<double, 80, 1> value; for (std::size_t index = 0; index < 80; ++index) value(index) = (*latest)[index]; robot_.set_par_DYN(value);
    }
    if (const auto* latest = par_REG_buffer_.readFromRT(); latest && latest->size() == 100) {
      Eigen::Matrix<double, 100, 1> value; for (std::size_t index = 0; index < 100; ++index) value(index) = (*latest)[index]; robot_.set_par_REG(value);
    }
    if (const auto* latest = q_buffer_.readFromRT(); latest && latest->size() == 7) {
      Eigen::Matrix<double, 7, 1> value; for (std::size_t index = 0; index < 7; ++index) value(index) = (*latest)[index]; robot_.set_q(value);
    }
    { FloatArray message; const auto value = robot_.get_Yr(); message.data = flatten_row_major(value); Yr_publisher_->publish(message); }
    { FloatArray message; const auto value = robot_.get_Y(); message.data = flatten_row_major(value); Y_publisher_->publish(message); }
    { FloatArray message; const auto value = robot_.get_reg_G(); message.data = flatten_row_major(value); reg_G_publisher_->publish(message); }
    { FloatArray message; const auto value = robot_.get_reg2dyn(); message.data = flatten_row_major(value); reg2dyn_publisher_->publish(message); }
    { FloatArray message; const auto value = robot_.get_q(); message.data = flatten_row_major(value); q_publisher_->publish(message); }
    cycle_start_us_.push_back(since_us(origin_, start));
    cycle_duration_us_.push_back(since_us(start, SteadyClock::now()));
  }

  thunder_franka robot_;
  std::string output_;
  SteadyClock::time_point origin_;
  std::vector<double> cycle_start_us_, cycle_duration_us_;
  rclcpp::TimerBase::SharedPtr timer_;
  realtime_tools::RealtimeBuffer<std::vector<float>> ddq_buffer_, ddqr_buffer_, dq_buffer_, dqr_buffer_, par_DYN_buffer_, par_REG_buffer_, q_buffer_;
  rclcpp::Subscription<FloatArray>::SharedPtr ddq_subscription_, ddqr_subscription_, dq_subscription_, dqr_subscription_, par_DYN_subscription_, par_REG_subscription_, q_subscription_;
  rclcpp::Service<franka_server::srv::GetValue>::SharedPtr get_par_REG_service_, get_Yr_service_, get_Y_service_;
  rclcpp::Publisher<FloatArray>::SharedPtr Yr_publisher_, Y_publisher_, reg_G_publisher_, reg2dyn_publisher_, q_publisher_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  sched_param priority{}; priority.sched_priority = 25;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &priority) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("franka_server"), "Unable to enable SCHED_FIFO priority 25; continuing normally");
  }
  {
    auto node = std::make_shared<frankaServer>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
