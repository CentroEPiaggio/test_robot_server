// Evaluation time of the generated functions the server exposes, on random
// configurations, outside ROS. Prints mean, p99 and max per function in us.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <thunder_franka/thunder_franka.h>

int main(int argc, char ** argv) {
  const int iterations = argc > 1 ? std::stoi(argv[1]) : 20000;
  thunder_franka robot;
  std::mt19937 generator(0);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  const auto random7 = [&]() { Eigen::Matrix<double, 7, 1> v; for (int i = 0; i < 7; ++i) v(i) = uniform(generator); return v; };

  volatile double sink = 0.0;
  const std::vector<std::pair<std::string, std::function<void()>>> functions = {
    {"T_w_EE", [&]() { sink = sink + robot.get_T_w_EE()(0, 0); }},
    {"J_EE", [&]() { sink = sink + robot.get_J_EE()(0, 0); }},
    {"M", [&]() { sink = sink + robot.get_M()(0, 0); }},
    {"C", [&]() { sink = sink + robot.get_C()(0, 0); }},
    {"G", [&]() { sink = sink + robot.get_G()(0); }},
    {"Yr", [&]() { sink = sink + robot.get_Yr()(0, 0); }},
    {"Y", [&]() { sink = sink + robot.get_Y()(0, 0); }},
    {"reg_G", [&]() { sink = sink + robot.get_reg_G()(0, 0); }},
    {"reg2dyn", [&]() { sink = sink + robot.get_reg2dyn()(0); }},
  };

  std::printf("function,mean_us,p50_us,p99_us,max_us\n");
  for (const auto & [name, function] : functions) {
    std::vector<double> durations(iterations);
    for (int k = 0; k < iterations; ++k) {
      robot.set_q(random7()); robot.set_dq(random7()); robot.set_ddq(random7());
      robot.set_dqr(random7()); robot.set_ddqr(random7());
      const auto start = std::chrono::steady_clock::now();
      function();
      durations[k] = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
    }
    std::vector<double> sorted = durations; std::sort(sorted.begin(), sorted.end());
    double mean = 0.0; for (double d : durations) mean += d; mean /= iterations;
    std::printf("%s,%.2f,%.2f,%.2f,%.2f\n", name.c_str(), mean, sorted[iterations / 2], sorted[static_cast<int>(0.99 * iterations)], sorted.back());
  }
  return 0;
}
