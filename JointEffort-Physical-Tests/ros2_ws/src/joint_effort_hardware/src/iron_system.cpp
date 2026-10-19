#include "joint_effort_hardware/sts_bus.hpp"
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <set>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/sysmacros.h>

namespace joint_effort_hardware {
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
class IronSystem : public hardware_interface::SystemInterface {
  Bus bus_;
  std::vector<Joint> joints_;
  std::vector<int> raw_;
  std::array<double, 5> position_{}, velocity_{}, command_{}, last_command_{};
  std::string port_;
  bool permitted_{false}, active_{false}, fault_{false}, fresh_{false};
  double min_voltage_{}, max_voltage_{}, max_temperature_{};
  Clock::time_point sampled_{};

  static double finite(const std::string & text) {
    size_t end{}; const double value=std::stod(text, &end);
    if (end != text.size() || !std::isfinite(value)) throw std::runtime_error("Invalid numeric parameter");
    return value;
  }
  static int integer(const std::string & text) {
    const double value=finite(text);
    if (value != std::floor(value) || value < -2147483647 || value > 2147483647)
      throw std::runtime_error("Invalid integer parameter");
    return static_cast<int>(value);
  }
  void sample() {
    const auto start=Clock::now();
    std::array<double, 5> positions{}, velocities{};
    std::vector<int> counts;
    for (size_t i=0; i<5; ++i) {
      const auto & j=joints_[i];
      // One block per motor: mode, torque, position, speed, voltage and temperature.
      const auto data=bus_.read(j.id, 33, 31);
      if (data[0] != 0 || (active_ && data[7] != 1))
        throw std::runtime_error(j.name+": position mode/torque changed");
      if (data[29]*.1 < min_voltage_ || data[29]*.1 > max_voltage_ || data[30] > max_temperature_)
        throw std::runtime_error(j.name+": voltage or temperature outside configured range");
      const int raw=signed_word(word(data, 23));
      positions[i]=j.radians(raw); counts.push_back(raw);
      velocities[i]=j.direction*signed_word(word(data,25))*6.283185307179586/(4096*j.ratio);
      if (active_ && std::abs(positions[i]-last_command_[i]) > j.tracking_error)
        throw std::runtime_error(j.name+": measured position does not follow the command");
    }
    if (std::chrono::duration<double>(Clock::now()-start).count() > .015)
      throw std::runtime_error("Five-motor feedback exceeded 15 ms; do not reduce this guard to hide bus latency");
    position_=positions; velocity_=velocities; raw_=counts; sampled_=start; fresh_=true;
  }
  void hold() {
    active_=false;
    if (fresh_ && raw_.size()==5 && std::chrono::duration<double>(Clock::now()-sampled_).count() < .05) {
      try { bus_.goals(joints_, raw_); }
      catch (const std::exception & e) { RCLCPP_ERROR(get_logger(), "Hold not acknowledged: %s", e.what()); }
    }
    // Keep torque as-is. A loaded arm can fall if its motors are released.
    RCLCPP_WARN(get_logger(), "Control stopped; motor torque was NOT released. Support arm before removing servo power.");
  }
  return_type fail(const std::exception & e, bool healthy_bus) {
    fault_=true;
    RCLCPP_ERROR(get_logger(), "Hardware fault latched: %s", e.what());
    if (healthy_bus) hold();
    else { active_=false; fresh_=false; }
    return return_type::ERROR;
  }
 public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
      return CallbackReturn::ERROR;
    try {
      const auto & p=info.hardware_parameters;
      const char * domain=std::getenv("ROS_DOMAIN_ID");
      const bool emulated=p.at("emulated_bus")=="true";
      if (!domain || std::string(domain)!=(emulated ? "90":"89")) throw std::runtime_error("Wrong ROS domain for physical/emulated hardware");
      for (const auto & key : {"calibration_confirmed", "motor_voltage_confirmed", "workspace_confirmed", "stop_arrangement_confirmed"})
        if (p.at(key)!="true") throw std::runtime_error(std::string(key)+" must be measured/confirmed");
      permitted_=p.at("allow_motor_writes")=="true";
      port_=p.at("serial_port");
      if (emulated) {
        struct stat device{};
        if (port_.rfind("/dev/pts/",0)!=0 || stat(port_.c_str(),&device)!=0 || !S_ISCHR(device.st_mode) ||
            major(device.st_rdev)<136 || major(device.st_rdev)>143)
          throw std::runtime_error("Emulation accepts only a Linux pseudo-terminal; real adapters forbidden");
      }
      if (port_.rfind("/dev/",0)!=0 || integer(p.at("baud_rate"))!=1000000)
        throw std::runtime_error("Actual /dev/ adapter and 1 Mbaud required");
      min_voltage_=finite(p.at("voltage_min_v")); max_voltage_=finite(p.at("voltage_max_v"));
      max_temperature_=finite(p.at("max_temperature_c"));
      if (!(min_voltage_>=3 && max_voltage_>min_voltage_ && max_voltage_<=14 && max_temperature_>=25 && max_temperature_<=60))
        throw std::runtime_error("Invalid voltage/temperature guards");
      if (info.joints.size()!=5) throw std::runtime_error("Exactly five joints required");
      std::set<int> ids;
      for (size_t i=0; i<5; ++i) {
        const auto & item=info.joints[i]; const auto & v=item.parameters;
        if (item.name!="joint_"+std::to_string(i+1) || item.command_interfaces.size()!=1 ||
            item.command_interfaces[0].name!="position" || item.state_interfaces.size()!=2 ||
            item.state_interfaces[0].name!="position" || item.state_interfaces[1].name!="velocity")
          throw std::runtime_error("Joint names/interfaces do not match the iron controller");
        Joint j{item.name,integer(v.at("motor_id")),integer(v.at("model_number")),
          integer(v.at("zero_counts")),integer(v.at("direction")),integer(v.at("min_counts")),integer(v.at("max_counts")),
          finite(v.at("gear_ratio")),finite(v.at("min_rad")),finite(v.at("max_rad")),
          finite(v.at("max_velocity_rad_s")),finite(v.at("max_tracking_error_rad"))};
        j.validate();
        if (!ids.insert(j.id).second) throw std::runtime_error("Duplicate motor ID");
        joints_.push_back(j);
      }
      position_.fill(std::numeric_limits<double>::quiet_NaN());
      velocity_=position_; command_=position_;
      return CallbackReturn::SUCCESS;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Hardware configuration refused: %s", e.what()); return CallbackReturn::ERROR;
    }
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> result;
    for (size_t i=0; i<5; ++i) {
      result.emplace_back(joints_[i].name,"position",&position_[i]);
      result.emplace_back(joints_[i].name,"velocity",&velocity_[i]);
    }
    return result;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> result;
    for (size_t i=0; i<5; ++i) result.emplace_back(joints_[i].name,"position",&command_[i]);
    return result;
  }
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    if (fault_) return CallbackReturn::ERROR;  // Restart and investigate; never auto-rearm a failed session.
    try {
      bus_.open(port_,1000000);
      for (const auto & j:joints_) {
        const auto identity=bus_.read(j.id,3,3);
        if (word(identity,0)!=j.model || identity[2]!=j.id) throw std::runtime_error(j.name+": wrong model or ID");
      }
      sample(); command_=last_command_=position_;
      RCLCPP_INFO(get_logger(),"Measured feedback ready. Hardware inactive; no motor writes sent.");
      return CallbackReturn::SUCCESS;
    } catch (const std::exception & e) { fail(e,false); bus_.close(); return CallbackReturn::ERROR; }
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    if (!permitted_ || fault_) {
      RCLCPP_ERROR(get_logger(),"Activation refused: explicit write permission and a fault-free session required");
      return CallbackReturn::FAILURE;
    }
    try {
      sample();
      for (const auto & j:joints_) {
        if (bus_.read(j.id,40,1)[0]!=0) throw std::runtime_error("Support the arm and arrange all motors torque-off before first activation");
      }
      // Preload the measured pose before enabling torque. Never send q=0 on startup.
      bus_.goals(joints_,raw_);
      for (size_t i=0; i<5; ++i) {
        const auto goal=bus_.read(joints_[i].id,42,2);
        if (word(goal,0)!=raw_[i]) throw std::runtime_error("Measured-pose preload readback failed");
      }
      for (size_t i=0; i<5; ++i) {
        if (std::abs(signed_word(word(bus_.read(joints_[i].id,56,2),0))-raw_[i])>1)
          throw std::runtime_error("Arm moved during preload; support it and restart activation");
      }
      for (const auto & j:joints_) bus_.torque_on(j.id);
      command_=last_command_=position_; active_=true; sample();
      return CallbackReturn::SUCCESS;
    } catch (const std::exception & e) { fail(e,true); return CallbackReturn::ERROR; }
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    hold(); return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    if (active_) hold();
    bus_.close(); fresh_=false; return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    if (active_) hold();
    bus_.close(); return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override {
    active_=false; bus_.close(); return CallbackReturn::ERROR;
  }
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    if (fault_) return return_type::ERROR;
    try { sample(); return return_type::OK; }
    catch (const std::exception & e) { return fail(e,false); }
  }
  return_type write(const rclcpp::Time &, const rclcpp::Duration & period) override {
    if (fault_) return return_type::ERROR;
    if (!active_) return return_type::OK;
    try {
      const double dt=period.seconds();
      if (!fresh_ || std::chrono::duration<double>(Clock::now()-sampled_).count()>.02 || dt<=0 || dt>.05)
        throw std::runtime_error("Control-loop deadline/feedback freshness failed");
      std::vector<int> targets;
      for (size_t i=0; i<5; ++i) {
        const auto & j=joints_[i];
        const int target=j.counts(command_[i]);
        j.check_step(command_[i],last_command_[i],position_[i],dt);
        targets.push_back(target);
      }
      bus_.goals(joints_,targets); last_command_=command_;
      return return_type::OK;
    } catch (const std::exception & e) { return fail(e,true); }
  }
};
}  // namespace joint_effort_hardware
PLUGINLIB_EXPORT_CLASS(joint_effort_hardware::IronSystem, hardware_interface::SystemInterface)
