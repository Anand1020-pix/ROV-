#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
std::atomic<bool> exit_requested{false};

void handle_signal(int)
{
  exit_requested.store(true);
}

class TerminalMode
{
public:
  explicit TerminalMode(int fd) : fd_(fd)
  {
    if (!isatty(fd_) || tcgetattr(fd_, &original_) != 0) {
      throw std::runtime_error("Teleop requires an interactive terminal");
    }

    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_iflag &= static_cast<tcflag_t>(~(IXON | ICRNL));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &raw) != 0) {
      throw std::runtime_error("Could not enable raw terminal input");
    }
    active_ = true;
  }

  ~TerminalMode()
  {
    if (active_) {
      tcsetattr(fd_, TCSANOW, &original_);
    }
  }

  TerminalMode(const TerminalMode &) = delete;
  TerminalMode & operator=(const TerminalMode &) = delete;

private:
  int fd_;
  termios original_{};
  bool active_{false};
};

class RovTeleop final : public rclcpp::Node
{
public:
  using SteadyClock = std::chrono::steady_clock;

  RovTeleop()
  : Node("rov_teleop")
  {
    declare_parameter<double>("linear_force_n", 1.0);
    declare_parameter<double>("pitch_torque_nm", 0.1);
    declare_parameter<double>("yaw_torque_nm", 0.1);

    linear_force_n_ = get_parameter("linear_force_n").as_double();
    pitch_torque_nm_ = get_parameter("pitch_torque_nm").as_double();
    yaw_torque_nm_ = get_parameter("yaw_torque_nm").as_double();
    if (!std::isfinite(linear_force_n_) || linear_force_n_ <= 0.0 ||
        !std::isfinite(pitch_torque_nm_) || pitch_torque_nm_ <= 0.0 ||
        !std::isfinite(yaw_torque_nm_) || yaw_torque_nm_ <= 0.0) {
      throw std::invalid_argument("Teleop force and torque parameters must be finite and positive");
    }

    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("/rov/cmd_wrench", 10);
    arm_pub_ = create_publisher<std_msgs::msg::Bool>("/rov/arm", 10);
    status_sub_ = create_subscription<std_msgs::msg::String>(
      "/rov/control_status", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if (msg->data == "ARMED") {
          controller_armed_ = true;
          arm_pending_ = false;
        } else if (msg->data == "DISARMED" || msg->data == "ACTIVE_DISARMED" ||
                   msg->data == "WATCHDOG_DISARMED" || msg->data == "RESET_DISARMED" ||
                   msg->data == "ESTOP_LATCHED") {
          controller_armed_ = false;
        }
      });
    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&RovTeleop::publish_tick, this));
    arm_at_ = SteadyClock::now();
  }

  bool handle_key(char key)
  {
    switch (key) {
      case 'w':
        desired_.force.x = linear_force_n_;
        break;
      case 's':
        desired_.force.x = -linear_force_n_;
        break;
      case 'a':
        desired_.force.y = linear_force_n_;
        break;
      case 'd':
        desired_.force.y = -linear_force_n_;
        break;
      case 'q':
        desired_.force.z = linear_force_n_;
        break;
      case 'e':
        desired_.force.z = -linear_force_n_;
        break;
      case 'i':
        desired_.torque.y = pitch_torque_nm_;
        break;
      case 'k':
        desired_.torque.y = -pitch_torque_nm_;
        break;
      case 'j':
        desired_.torque.z = yaw_torque_nm_;
        break;
      case 'l':
        desired_.torque.z = -yaw_torque_nm_;
        break;
      case ' ':
        stop();
        return true;
      case '\x1b':
        stop();
        return false;
      default:
        return true;
    }

    if (!controller_armed_ && !arm_pending_) {
      arm_pending_ = true;
      arm_attempts_ = 0;
      arm_at_ = SteadyClock::now() + std::chrono::milliseconds(150);
    }
    std::cout << "\rCommand updated. Axes remain active until reversed or SPACE is pressed. "
              << std::flush;
    return true;
  }

  void publish_tick()
  {
    publish_wrench();
    if (!announced_) {
      RCLCPP_INFO(get_logger(), "Teleop publishing wrench commands at 20 Hz");
      announced_ = true;
    }
    if (arm_pending_ && !controller_armed_ && SteadyClock::now() >= arm_at_) {
      RCLCPP_INFO(get_logger(), "Requesting thruster controller arm");
      publish_arm(true);
      ++arm_attempts_;
      if (arm_attempts_ >= max_arm_attempts_) {
        arm_pending_ = false;
        std::cout << "\rCould not confirm arming; check controller state/E-stop, then press a movement key again. "
                  << std::flush;
      } else {
        arm_at_ = SteadyClock::now() + std::chrono::milliseconds(300);
      }
    }
  }

  void stop()
  {
    desired_ = geometry_msgs::msg::Wrench();
    arm_pending_ = false;
    arm_attempts_ = 0;
    controller_armed_ = false;
    publish_wrench();
    publish_arm(false);
    std::cout << "\rSTOP: command cleared and controller disarmed.             " << std::flush;
  }

  void flush_stop(rclcpp::Executor & executor)
  {
    for (int i = 0; i < 5; ++i) {
      stop();
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::cout << '\n';
  }

private:
  void publish_wrench()
  {
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.wrench = desired_;
    wrench_pub_->publish(msg);
  }

  void publish_arm(bool arm)
  {
    std_msgs::msg::Bool msg;
    msg.data = arm;
    arm_pub_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  geometry_msgs::msg::Wrench desired_{};
  SteadyClock::time_point arm_at_{};
  double linear_force_n_{1.0};
  double pitch_torque_nm_{0.1};
  double yaw_torque_nm_{0.1};
  bool arm_pending_{false};
  bool controller_armed_{false};
  bool announced_{false};
  int arm_attempts_{0};
  static constexpr int max_arm_attempts_{4};
};

void print_help()
{
  std::cout
    << "ROV keyboard teleop (body frame, base_link)\n"
    << "  W / S : forward / reverse\n"
    << "  A / D : left / right\n"
    << "  Q / E : up / down\n"
    << "  I / K : pitch + / pitch -\n"
    << "  J / L : yaw left / yaw right\n"
    << "  SPACE : STOP (clear commands and disarm)\n"
    << "  ESC or Ctrl+C : stop and exit\n"
    << "Axes latch until reversed; pressing multiple axes combines motion.\n"
    << "Press a movement key to arm. Keep this terminal visible and use SPACE to stop.\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    TerminalMode terminal(STDIN_FILENO);
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<RovTeleop>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    print_help();

    bool running = true;
    while (rclcpp::ok() && !exit_requested.load() && running) {
      pollfd input{STDIN_FILENO, POLLIN, 0};
      const int ready = poll(&input, 1, 10);
      if (ready < 0 && errno != EINTR) {
        throw std::runtime_error("Terminal input polling failed");
      }
      if (ready > 0 && (input.revents & POLLIN) != 0) {
        char key = 0;
        while (read(STDIN_FILENO, &key, 1) == 1) {
          if (key >= 'A' && key <= 'Z') {
            key = static_cast<char>(key - 'A' + 'a');
          }
          running = node->handle_key(key);
          if (!running) {
            break;
          }
        }
      }

      executor.spin_some();
    }

    node->flush_stop(executor);
    executor.remove_node(node);
    rclcpp::shutdown();
  } catch (const std::exception & error) {
    std::cerr << "rov_teleop: " << error.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  return 0;
}
