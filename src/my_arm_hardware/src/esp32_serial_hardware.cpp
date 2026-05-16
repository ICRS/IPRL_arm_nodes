#include "my_arm_hardware/esp32_serial_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace my_arm_hardware
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

Esp32SerialHardware::Esp32SerialHardware()
: port_("/dev/ttyUSB0"),
  baud_rate_(115200),
  real_joints_(3),
  id_offset_(0),
  query_period_ms_(50),
  ping_on_activate_(true),
  ping_id_(50),
  ping_timeout_ms_(300),
  poll_cur_ang_(false),
  stream_list_mode_(true),
  stream_without_id_(false),
  wait_for_first_state_(false),
  state_timeout_ms_(500),
  joint_1_min_deg_(0.0),
  joint_1_max_deg_(360.0),
  joint_2_min_deg_(0.0),
  joint_2_max_deg_(160.0),
  joint_1_offset_deg_(0.0),
  joint_2_offset_deg_(0.0),
  joint_3_offset_deg_(0.0),
  joint_4_offset_deg_(0.0),
  logger_(rclcpp::get_logger("my_arm_hardware")),
  serial_(io_),
  stop_requested_(false),
  connected_(false),
  last_pong_id_(-1),
  stream_joint_index_(0)
{
}

hardware_interface::CallbackReturn Esp32SerialHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_ = get_param_string("port", port_);
  baud_rate_ = get_param_int("baud_rate", baud_rate_);
  real_joints_ = get_param_int("real_joints", real_joints_);
  id_offset_ = get_param_int("id_offset", id_offset_);
  query_period_ms_ = get_param_int("query_period_ms", query_period_ms_);
  ping_on_activate_ = get_param_bool("ping_on_activate", ping_on_activate_);
  ping_id_ = get_param_int("ping_id", ping_id_);
  ping_timeout_ms_ = get_param_int("ping_timeout_ms", ping_timeout_ms_);
  poll_cur_ang_ = get_param_bool("poll_cur_ang", poll_cur_ang_);
  stream_list_mode_ = get_param_bool("stream_list_mode", stream_list_mode_);
  stream_without_id_ = get_param_bool("stream_without_id", stream_without_id_);
  wait_for_first_state_ = get_param_bool("wait_for_first_state", wait_for_first_state_);
  state_timeout_ms_ = get_param_int("state_timeout_ms", state_timeout_ms_);
  joint_1_min_deg_ = get_param_double("joint_1_min_deg", joint_1_min_deg_);
  joint_1_max_deg_ = get_param_double("joint_1_max_deg", joint_1_max_deg_);
  joint_2_min_deg_ = get_param_double("joint_2_min_deg", joint_2_min_deg_);
  joint_2_max_deg_ = get_param_double("joint_2_max_deg", joint_2_max_deg_);
  joint_1_offset_deg_ = get_param_double("joint_1_offset_deg", joint_1_offset_deg_);
  joint_2_offset_deg_ = get_param_double("joint_2_offset_deg", joint_2_offset_deg_);
  joint_3_offset_deg_ = get_param_double("joint_3_offset_deg", joint_3_offset_deg_);
  joint_4_offset_deg_ = get_param_double("joint_4_offset_deg", joint_4_offset_deg_);

  if (real_joints_ < 0) {
    real_joints_ = 0;
  }
  if (static_cast<size_t>(real_joints_) > info_.joints.size()) {
    real_joints_ = static_cast<int>(info_.joints.size());
  }
  if (id_offset_ < 0) {
    id_offset_ = 0;
  }
  if (state_timeout_ms_ < 0) {
    state_timeout_ms_ = 0;
  }

  hw_states_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);
  hw_commands_.assign(info_.joints.size(), 0.0);

  joint_ids_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    joint_ids_[i] = static_cast<int>(i) + id_offset_;
    RCLCPP_INFO(
      logger_, "Joint '%s' mapped to ID %d",
      info_.joints[i].name.c_str(), joint_ids_[i]);
  }

  latest_joint_deg_.assign(info_.joints.size() + 1, 0.0);
  latest_joint_deg_valid_.assign(info_.joints.size() + 1, false);
  last_query_time_ = std::chrono::steady_clock::now();
  last_pong_id_ = -1;
  stream_joint_index_ = 0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Esp32SerialHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & interface : info_.joints[i].state_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        state_interfaces.emplace_back(info_.joints[i].name, interface.name, &hw_states_[i]);
      } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        state_interfaces.emplace_back(info_.joints[i].name, interface.name, &hw_velocities_[i]);
      } else {
        RCLCPP_WARN(
          logger_, "Unsupported state interface '%s' for joint '%s'",
          interface.name.c_str(), info_.joints[i].name.c_str());
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Esp32SerialHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & interface : info_.joints[i].command_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        command_interfaces.emplace_back(info_.joints[i].name, interface.name, &hw_commands_[i]);
      } else {
        RCLCPP_WARN(
          logger_, "Unsupported command interface '%s' for joint '%s'",
          interface.name.c_str(), info_.joints[i].name.c_str());
      }
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Esp32SerialHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32SerialHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!open_serial()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  start_read_thread();
  last_query_time_ = std::chrono::steady_clock::now();

  if (ping_on_activate_) {
    last_pong_id_ = -1;
    std::ostringstream ping;
    ping << "<PING:" << ping_id_ << ">";
    send_line(ping.str());

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(ping_timeout_ms_);
    while (std::chrono::steady_clock::now() < deadline) {
      consume_rx_queue();
      if (last_pong_id_ == ping_id_) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (last_pong_id_ != ping_id_) {
      RCLCPP_ERROR(logger_, "No PONG response from ESP32. Activation failed.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (wait_for_first_state_ && real_joints_ > 0 && state_timeout_ms_ > 0) {
    auto have_states = [this]() {
      for (int i = 0; i < real_joints_; ++i) {
        const int id = joint_ids_[i];
        if (id < 0 || static_cast<size_t>(id) >= latest_joint_deg_valid_.size()) {
          return false;
        }
        if (!latest_joint_deg_valid_[id]) {
          return false;
        }
      }
      return true;
    };

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(state_timeout_ms_);
    while (std::chrono::steady_clock::now() < deadline) {
      consume_rx_queue();
      if (have_states()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!have_states()) {
      RCLCPP_WARN(logger_, "No joint state stream received before timeout.");
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32SerialHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  stop_read_thread();
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Esp32SerialHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!connected_) {
    return hardware_interface::return_type::ERROR;
  }

  consume_rx_queue();

  if (poll_cur_ang_) {
    const auto now = std::chrono::steady_clock::now();
    const auto query_period = std::chrono::milliseconds(query_period_ms_);
    bool should_query = false;
    if (query_period_ms_ <= 0) {
      should_query = true;
    } else if (now - last_query_time_ >= query_period) {
      should_query = true;
    }

    if (should_query) {
      last_query_time_ = now;
      for (int i = 0; i < real_joints_; ++i) {
        std::ostringstream request;
        request << "<CUR_ANG:" << joint_ids_[i] << ">";
        send_line(request.str());
        pending_angle_requests_.push_back(joint_ids_[i]);
      }
    }
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (static_cast<int>(i) < real_joints_) {
      const int id = joint_ids_[i];
      if (id >= 0 && static_cast<size_t>(id) < latest_joint_deg_.size() &&
        latest_joint_deg_valid_[id])
      {
        hw_states_[i] = deg_to_rad(latest_joint_deg_[id] + get_joint_offset_deg(i));
      }
    } else {
      hw_states_[i] = hw_commands_[i];
    }
    hw_velocities_[i] = 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Esp32SerialHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!connected_) {
    return hardware_interface::return_type::ERROR;
  }

  for (int i = 0; i < real_joints_; ++i) {
    double command_deg = rad_to_deg(hw_commands_[i]);
    command_deg -= get_joint_offset_deg(i);
    if (i == 0) {
      command_deg = std::min(std::max(command_deg, joint_1_min_deg_), joint_1_max_deg_);
    } else if (i == 1) {
      command_deg = std::min(std::max(command_deg, joint_2_min_deg_), joint_2_max_deg_);
    }

    std::ostringstream command;
    command << "<DES_VAL:" << joint_ids_[i] << "," << std::fixed << std::setprecision(3)
            << command_deg << ">";
    send_line(command.str());
  }

  return hardware_interface::return_type::OK;
}

bool Esp32SerialHardware::open_serial()
{
  boost::system::error_code ec;
  serial_.open(port_, ec);
  if (ec) {
    RCLCPP_ERROR(logger_, "Failed to open serial port %s: %s", port_.c_str(), ec.message().c_str());
    connected_ = false;
    return false;
  }

  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  serial_.set_option(boost::asio::serial_port_base::character_size(8));
  serial_.set_option(boost::asio::serial_port_base::parity(
    boost::asio::serial_port_base::parity::none));
  serial_.set_option(boost::asio::serial_port_base::stop_bits(
    boost::asio::serial_port_base::stop_bits::one));
  serial_.set_option(boost::asio::serial_port_base::flow_control(
    boost::asio::serial_port_base::flow_control::none));

  connected_ = true;
  return true;
}

void Esp32SerialHardware::close_serial()
{
  boost::system::error_code ec;
  if (serial_.is_open()) {
    serial_.cancel(ec);
    serial_.close(ec);
  }
  connected_ = false;
}

void Esp32SerialHardware::start_read_thread()
{
  stop_requested_ = false;
  read_thread_ = std::thread([this]() {
    boost::asio::streambuf buffer;
    while (!stop_requested_) {
      boost::system::error_code ec;
      boost::asio::read_until(serial_, buffer, '\n', ec);
      if (ec) {
        if (!stop_requested_) {
          RCLCPP_WARN(logger_, "Serial read error: %s", ec.message().c_str());
        }
        break;
      }

      std::istream stream(&buffer);
      std::string line;
      std::getline(stream, line);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }

      std::lock_guard<std::mutex> lock(rx_mutex_);
      rx_queue_.push_back(line);
    }
    connected_ = false;
  });
}

void Esp32SerialHardware::stop_read_thread()
{
  stop_requested_ = true;
  boost::system::error_code ec;
  if (serial_.is_open()) {
    serial_.cancel(ec);
  }
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

bool Esp32SerialHardware::send_line(const std::string & line)
{
  if (!connected_ || !serial_.is_open()) {
    return false;
  }

  const std::string payload = line + "\n";
  std::lock_guard<std::mutex> lock(tx_mutex_);
  boost::system::error_code ec;
  boost::asio::write(serial_, boost::asio::buffer(payload), ec);
  if (ec) {
    RCLCPP_WARN(logger_, "Serial write error: %s", ec.message().c_str());
    connected_ = false;
    return false;
  }

  return true;
}

void Esp32SerialHardware::consume_rx_queue()
{
  std::deque<std::string> local_queue;
  {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    local_queue.swap(rx_queue_);
  }

  for (const auto & line : local_queue) {
    parse_message(line);
  }
}

void Esp32SerialHardware::parse_message(const std::string & line)
{
  std::string message = trim_copy(line);
  if (message.empty()) {
    return;
  }

  if (message.front() == '<' && message.back() == '>') {
    message = message.substr(1, message.size() - 2);
  }

  const auto colon_pos = message.find(':');
  if (colon_pos == std::string::npos) {
    return;
  }

  const std::string key = to_upper_copy(message.substr(0, colon_pos));
  const std::string value = trim_copy(message.substr(colon_pos + 1));

  if (key == "CUR_ANG") {
    const size_t comma_count = static_cast<size_t>(std::count(value.begin(), value.end(), ','));
    if (stream_list_mode_ && comma_count >= 2) {
      std::vector<double> values;
      std::stringstream stream(value);
      std::string token;
      while (std::getline(stream, token, ',')) {
        const std::string trimmed = trim_copy(token);
        if (trimmed.empty()) {
          continue;
        }
        try {
          values.push_back(std::stod(trimmed));
        } catch (const std::exception &) {
          return;
        }
      }

      const size_t update_count = std::min(values.size(), static_cast<size_t>(real_joints_));
      for (size_t i = 0; i < update_count; ++i) {
        const int id = joint_ids_[i];
        if (id >= 0 && static_cast<size_t>(id) < latest_joint_deg_.size()) {
          latest_joint_deg_[id] = values[i];
          latest_joint_deg_valid_[id] = true;
        }
      }

      if (!pending_angle_requests_.empty()) {
        pending_angle_requests_.clear();
      }
      return;
    }

    int joint_id = -1;
    double angle_deg = 0.0;
    const auto comma_pos = value.find(',');

    try {
      if (comma_pos != std::string::npos) {
        joint_id = std::stoi(trim_copy(value.substr(0, comma_pos)));
        angle_deg = std::stod(trim_copy(value.substr(comma_pos + 1)));
        const auto pending_it = std::find(
          pending_angle_requests_.begin(), pending_angle_requests_.end(), joint_id);
        if (pending_it != pending_angle_requests_.end()) {
          pending_angle_requests_.erase(pending_it);
        }
      } else {
        angle_deg = std::stod(value);
        if (!pending_angle_requests_.empty()) {
          joint_id = pending_angle_requests_.front();
          pending_angle_requests_.pop_front();
        } else if (stream_without_id_ && real_joints_ > 0) {
          joint_id = joint_ids_[stream_joint_index_];
          stream_joint_index_ = (stream_joint_index_ + 1) % real_joints_;
        }
      }
    } catch (const std::exception &) {
      return;
    }

    if (joint_id >= 0 && static_cast<size_t>(joint_id) < latest_joint_deg_.size()) {
      latest_joint_deg_[joint_id] = angle_deg;
      latest_joint_deg_valid_[joint_id] = true;
    }
  } else if (key == "PONG") {
    try {
      last_pong_id_ = std::stoi(value);
    } catch (const std::exception &) {
      last_pong_id_ = -1;
    }
  }
}

std::string Esp32SerialHardware::trim_copy(const std::string & value) const
{
  size_t start = 0;
  while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start]))) {
    ++start;
  }
  size_t end = value.size();
  while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }
  return value.substr(start, end - start);
}

std::string Esp32SerialHardware::to_upper_copy(const std::string & value) const
{
  std::string result = value;
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return result;
}

std::string Esp32SerialHardware::get_param_string(
  const std::string & key, const std::string & default_value) const
{
  const auto it = info_.hardware_parameters.find(key);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }
  return it->second;
}

int Esp32SerialHardware::get_param_int(const std::string & key, int default_value) const
{
  const auto it = info_.hardware_parameters.find(key);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }

  try {
    return std::stoi(it->second);
  } catch (const std::exception &) {
    return default_value;
  }
}

double Esp32SerialHardware::get_param_double(
  const std::string & key, double default_value) const
{
  const auto it = info_.hardware_parameters.find(key);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }

  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    return default_value;
  }
}

bool Esp32SerialHardware::get_param_bool(const std::string & key, bool default_value) const
{
  const auto it = info_.hardware_parameters.find(key);
  if (it == info_.hardware_parameters.end()) {
    return default_value;
  }

  std::string value = to_upper_copy(trim_copy(it->second));
  if (value == "TRUE" || value == "1" || value == "YES" || value == "ON") {
    return true;
  }
  if (value == "FALSE" || value == "0" || value == "NO" || value == "OFF") {
    return false;
  }
  return default_value;
}

double Esp32SerialHardware::rad_to_deg(double rad) const
{
  return rad * 180.0 / kPi;
}

double Esp32SerialHardware::deg_to_rad(double deg) const
{
  return deg * kPi / 180.0;
}

double Esp32SerialHardware::get_joint_offset_deg(size_t index) const
{
  switch (index) {
    case 0:
      return joint_1_offset_deg_;
    case 1:
      return joint_2_offset_deg_;
    case 2:
      return joint_3_offset_deg_;
    case 3:
      return joint_4_offset_deg_;
    default:
      return 0.0;
  }
}

}  // namespace my_arm_hardware

PLUGINLIB_EXPORT_CLASS(my_arm_hardware::Esp32SerialHardware, hardware_interface::SystemInterface)
