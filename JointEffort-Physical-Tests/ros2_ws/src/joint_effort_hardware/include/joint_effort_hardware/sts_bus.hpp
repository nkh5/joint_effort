#pragma once
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace joint_effort_hardware {
using Bytes = std::vector<uint8_t>;
using Clock = std::chrono::steady_clock;

Bytes packet(int id, int instruction, const Bytes & parameters);
int word(const Bytes & data, size_t offset);
int signed_word(int raw);

struct Joint {
  std::string name;
  int id{}, model{}, zero{}, direction{}, min_counts{}, max_counts{};
  double ratio{}, lower{}, upper{}, velocity{}, tracking_error{};
  void validate() const;
  double radians(int counts) const;
  int counts(double radians) const;
  int speed() const;
  void check_step(double command, double previous, double measured, double period) const;
};

// One owner, bounded transactions, no automatic retries or EEPROM writes.
class Bus {
 public:
  Bus() = default;
  ~Bus();
  Bus(const Bus &) = delete;
  Bus & operator=(const Bus &) = delete;
  void open(const std::string & path, int baud);
  void close();
  Bytes read(int id, int address, int length);
  void goals(const std::vector<Joint> & joints, const std::vector<int> & targets);
  void torque_on(int id);
 private:
  int fd_{-1};
  void send(const Bytes & frame);
  Bytes response(int id, int length);
};
}  // namespace joint_effort_hardware
