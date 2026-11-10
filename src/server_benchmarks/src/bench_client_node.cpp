// A stand-in for the MACT server controller, without the robot.
//
// Every millisecond it publishes the same inputs the controller publishes, with
// a request counter encoded in q[0], and then measures one of:
//
//   mode = topic    the round trip from publishing a state to receiving the
//                   server's q topic carrying it back (published after Yr, Y,
//                   reg_G and reg2dyn in the same server cycle), and the age
//                   of the newest Yr at every tick, as the controller sees it;
//   mode = service  the round trip of a get_Yr call, with at most one call in
//                   flight; a tick with a call still pending is a miss.
//
// Results are written to a CSV on shutdown.

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <pthread.h>
#include <sched.h>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <franka_server/msg/float32_array.hpp>
#include <franka_server/srv/get_value.hpp>

namespace {
using FloatArray = franka_server::msg::Float32Array;
using GetValue = franka_server::srv::GetValue;
using SteadyClock = std::chrono::steady_clock;
constexpr double kCounterStep = 1e-3;  // q[0] = counter * step, exact to decode in float32
double us_between(SteadyClock::time_point from, SteadyClock::time_point to) {
  return std::chrono::duration<double, std::micro>(to - from).count();
}
}  // namespace

class BenchClient : public rclcpp::Node {
public:
  BenchClient() : Node("bench_client") {
    mode_ = declare_parameter<std::string>("mode", "topic");
    output_ = declare_parameter<std::string>("output", "client.csv");
    const auto max_requests = declare_parameter<int>("max_requests", 60000);
    send_time_.resize(max_requests + 1);
    received_.assign(max_requests + 1, false);
    samples_.reserve(max_requests);

    for (const auto & name : {"q", "dq", "ddq", "dqr", "ddqr"}) {
      publishers_.push_back(create_publisher<FloatArray>(std::string("/controller/") + name, rclcpp::QoS(1)));
    }
    par_reg_publisher_ = create_publisher<FloatArray>("/controller/par_REG", rclcpp::QoS(1));
    par_dyn_publisher_ = create_publisher<FloatArray>("/controller/par_DYN", rclcpp::QoS(1));
    state_.data.assign(7, 0.1F);
    par_reg_.data.assign(100, 0.0F);
    par_dyn_.data.assign(80, 0.0F);

    // Same QoS as the controller: best effort, keep last one.
    yr_subscription_ = create_subscription<FloatArray>(
      "franka_server/Yr", rclcpp::QoS(1).best_effort(),
      [this](FloatArray::SharedPtr) { last_yr_.writeFromNonRT(SteadyClock::now()); });
    echo_subscription_ = create_subscription<FloatArray>(
      "franka_server/q", rclcpp::QoS(1).best_effort(),
      [this](FloatArray::SharedPtr message) { on_echo(*message); });

    client_ = create_client<GetValue>("franka_server/get_Yr");
    timer_ = create_wall_timer(std::chrono::microseconds(1000), [this]() { on_tick(); });
  }

  ~BenchClient() override {
    std::ofstream file(output_);
    file << "kind,value_us\n";
    for (const auto & [kind, value] : samples_) file << kind << "," << value << "\n";
    file << "summary_requests," << counter_ << "\n";
    file << "summary_missed_ticks," << missed_ticks_ << "\n";
  }

private:
  void on_tick() {
    const auto now = SteadyClock::now();
    if (counter_ + 1 >= static_cast<long>(send_time_.size())) return;

    // Age of the newest Yr, as the controller computes it at each cycle.
    if (const auto * last = last_yr_.readFromRT(); last && last->time_since_epoch().count() > 0) {
      samples_.emplace_back("age", us_between(*last, now));
    }

    ++counter_;
    send_time_[counter_] = now;
    state_.data[0] = static_cast<float>(counter_ * kCounterStep);
    publishers_[0]->publish(state_);
    state_.data[0] = 0.1F;
    for (std::size_t index = 1; index < publishers_.size(); ++index) publishers_[index]->publish(state_);
    par_reg_publisher_->publish(par_reg_);
    par_dyn_publisher_->publish(par_dyn_);

    if (mode_ == "service") {
      if (pending_) { ++missed_ticks_; return; }
      if (!client_->service_is_ready()) return;
      pending_ = true;
      const auto sent = SteadyClock::now();
      client_->async_send_request(
        std::make_shared<GetValue::Request>(),
        [this, sent](rclcpp::Client<GetValue>::SharedFuture) {
          samples_.emplace_back("service_rtt", us_between(sent, SteadyClock::now()));
          pending_ = false;
        });
    }
  }

  void on_echo(const FloatArray & message) {
    if (message.data.empty()) return;
    const long counter = std::lround(message.data[0] / kCounterStep);
    if (counter <= 0 || counter > counter_ || received_[counter]) return;
    received_[counter] = true;
    samples_.emplace_back("topic_rtt", us_between(send_time_[counter], SteadyClock::now()));
  }

  std::string mode_, output_;
  long counter_ = 0, missed_ticks_ = 0;
  bool pending_ = false;
  std::vector<SteadyClock::time_point> send_time_;
  std::vector<bool> received_;
  std::vector<std::pair<std::string, double>> samples_;
  FloatArray state_, par_reg_, par_dyn_;
  realtime_tools::RealtimeBuffer<SteadyClock::time_point> last_yr_;
  std::vector<rclcpp::Publisher<FloatArray>::SharedPtr> publishers_;
  rclcpp::Publisher<FloatArray>::SharedPtr par_reg_publisher_, par_dyn_publisher_;
  rclcpp::Subscription<FloatArray>::SharedPtr yr_subscription_, echo_subscription_;
  rclcpp::Client<GetValue>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  // Above the server, as the controller manager is.
  sched_param priority{}; priority.sched_priority = 50;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &priority) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("bench_client"), "Unable to enable SCHED_FIFO priority 50");
  }
  {
    auto node = std::make_shared<BenchClient>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
