#ifndef MY_ARM_HARDWARE__ESP32_SERIAL_HARDWARE_HPP_
#define MY_ARM_HARDWARE__ESP32_SERIAL_HARDWARE_HPP_

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace my_arm_hardware
{

class Esp32SerialHardware : public hardware_interface::SystemInterface
{
public:
  Esp32SerialHardware();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  bool open_serial();
  void close_serial();
  void start_read_thread();
  void stop_read_thread();
  bool send_line(const std::string & line);
  void consume_rx_queue();
  void parse_message(const std::string & line);
  std::string trim_copy(const std::string & value) const;
  std::string to_upper_copy(const std::string & value) const;

  std::string get_param_string(const std::string & key, const std::string & default_value) const;
  int get_param_int(const std::string & key, int default_value) const;
  double get_param_double(const std::string & key, double default_value) const;
  bool get_param_bool(const std::string & key, bool default_value) const;

  double rad_to_deg(double rad) const;
  double deg_to_rad(double deg) const;
  double get_joint_offset_deg(size_t index) const;

  std::string port_;
  int baud_rate_;
  int real_joints_;
  int id_offset_;
  int query_period_ms_;
  bool ping_on_activate_;
  int ping_id_;
  int ping_timeout_ms_;
  bool poll_cur_ang_;
  bool stream_list_mode_;
  bool stream_without_id_;
  bool wait_for_first_state_;
  int state_timeout_ms_;
  double joint_1_min_deg_;
  double joint_1_max_deg_;
  double joint_2_min_deg_;
  double joint_2_max_deg_;
  double joint_1_offset_deg_;
  double joint_2_offset_deg_;
  double joint_3_offset_deg_;
  double joint_4_offset_deg_;

  rclcpp::Logger logger_;

  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
  std::thread read_thread_;
  std::atomic<bool> stop_requested_;
  std::atomic<bool> connected_;
  std::mutex rx_mutex_;
  std::deque<std::string> rx_queue_;
  std::mutex tx_mutex_;

  std::vector<double> hw_states_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<int> joint_ids_;
  std::vector<double> latest_joint_deg_;
  std::vector<bool> latest_joint_deg_valid_;
  std::deque<int> pending_angle_requests_;
  std::chrono::steady_clock::time_point last_query_time_;
  int last_pong_id_;
  int stream_joint_index_;
};

}  // namespace my_arm_hardware

#endif  // MY_ARM_HARDWARE__ESP32_SERIAL_HARDWARE_HPP_
