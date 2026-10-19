#include "joint_effort_hardware/sts_bus.hpp"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace joint_effort_hardware {
namespace {
constexpr double tau = 6.2831853071795864769;
void require(bool ok, const std::string & message) {
  if (!ok) throw std::runtime_error(message);
}
}
Bytes packet(int id, int instruction, const Bytes & parameters) {
  require(id >= 1 && id <= 254 && parameters.size() <= 250, "Invalid STS packet");
  Bytes out{255, 255, static_cast<uint8_t>(id), static_cast<uint8_t>(parameters.size()+2),
            static_cast<uint8_t>(instruction)};
  out.insert(out.end(), parameters.begin(), parameters.end());
  unsigned sum = 0;
  for (size_t i=2; i<out.size(); ++i) sum += out[i];
  out.push_back(static_cast<uint8_t>(~sum));
  return out;
}
int word(const Bytes & data, size_t offset) {
  return data.at(offset) | data.at(offset+1) << 8;
}
int signed_word(int raw) { return raw & 0x8000 ? -(raw & 0x7fff) : raw; }
void Joint::validate() const {
  require(id >= 1 && id <= 253 && model >= 0 && model <= 65535,
          name+": invalid ID/model");
  require(min_counts >= 0 && min_counts < max_counts && max_counts <= 4095 &&
          zero >= min_counts && zero <= max_counts && (direction == 1 || direction == -1),
          name+": invalid count limits, zero or direction");
  require(std::isfinite(ratio) && ratio > 0 && ratio <= 100 &&
          std::isfinite(lower) && std::isfinite(upper) && lower < upper &&
          std::isfinite(velocity) && velocity > 0 && velocity <= .15 &&
          std::isfinite(tracking_error) && tracking_error >= tau/(4096*ratio) && tracking_error <= .05,
          name+": invalid angle/speed/tracking limits");
  counts(lower); counts(upper);
  require(speed() >= 1 && speed() <= 32767, name+": invalid native speed");
}
double Joint::radians(int raw) const {
  require(raw >= min_counts && raw <= max_counts, name+": encoder outside measured count limits");
  const double q = direction*(raw-zero)*tau/(4096*ratio);
  require(q >= lower-1e-12 && q <= upper+1e-12, name+": measured angle outside limits");
  return q;
}
int Joint::counts(double q) const {
  require(std::isfinite(q) && q >= lower && q <= upper, name+": requested angle outside limits");
  const double raw = zero + direction*q*4096*ratio/tau;
  require(raw >= min_counts && raw <= max_counts, name+": target would wrap or exceed count limits");
  const double end_a=zero+direction*lower*4096*ratio/tau;
  const double end_b=zero+direction*upper*4096*ratio/tau;
  const int low=std::max(min_counts,static_cast<int>(std::ceil(std::min(end_a,end_b)-1e-10)));
  const int high=std::min(max_counts,static_cast<int>(std::floor(std::max(end_a,end_b)+1e-10)));
  require(low <= high, name+": no encoder count fits the angle range");
  const int target = std::clamp(static_cast<int>(std::floor(raw+.5)),low,high);
  // Round inward at the limits; one encoder count must not cross a measured boundary.
  radians(target);
  return target;
}
int Joint::speed() const { return static_cast<int>(std::floor(velocity*4096*ratio/tau)); }
void Joint::check_step(double command, double previous, double measured, double period) const {
  counts(command);
  const double quantum=tau/(4096*ratio);
  // Servo rebases on measured encoder angles when it takes control. Allow one
  // count of rounding; the motor's native speed limit still caps actual motion.
  require(std::isfinite(previous) && std::isfinite(measured) && std::isfinite(period) && period>0 && period<=.05 &&
          std::abs(command-previous)<=velocity*period+quantum+1e-8 &&
          std::abs(command-measured)<=tracking_error,
          name+": discontinuous command or following-error limit; command="+std::to_string(command)+
          ", previous="+std::to_string(previous)+", measured="+std::to_string(measured)+", period="+std::to_string(period));
}
Bus::~Bus() { close(); }
void Bus::close() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}
void Bus::open(const std::string & path, int baud) {
  require(fd_ == -1, "Serial bus already open");
  require(baud == 1000000, "This driver currently supports only verified 1 Mbaud operation");
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  require(fd_ >= 0, "Cannot open serial adapter: "+std::string(std::strerror(errno)));
  try {
    require(flock(fd_, LOCK_EX | LOCK_NB) == 0, "Another process owns the serial adapter");
    require(ioctl(fd_, TIOCEXCL) == 0, "Cannot obtain exclusive serial access");
    termios options{};
    require(tcgetattr(fd_, &options) == 0, "Cannot read serial settings");
    cfmakeraw(&options); options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~(CSTOPB | CRTSCTS); options.c_cc[VMIN] = 0; options.c_cc[VTIME] = 0;
    cfsetispeed(&options, B1000000); cfsetospeed(&options, B1000000);
    require(tcsetattr(fd_, TCSANOW, &options) == 0 && tcflush(fd_, TCIOFLUSH) == 0,
            "Cannot set 1 Mbaud 8N1 serial settings");
  } catch (...) { close(); throw; }
}
void Bus::send(const Bytes & frame) {
  require(fd_ >= 0, "Serial bus is closed");
  size_t sent = 0;
  const auto deadline = Clock::now()+std::chrono::milliseconds(5);
  while (sent < frame.size()) {
    const auto n = ::write(fd_, frame.data()+sent, frame.size()-sent);
    if (n > 0) sent += static_cast<size_t>(n);
    else if (n < 0 && errno != EAGAIN && errno != EINTR) throw std::runtime_error("Serial write failed");
    require(Clock::now() < deadline, "Serial write deadline missed");
    if (sent < frame.size()) { pollfd p{fd_, POLLOUT, 0}; ::poll(&p, 1, 1); }
  }
}
Bytes Bus::response(int id, int length) {
  Bytes result;
  const auto deadline = Clock::now()+std::chrono::milliseconds(8);
  while (Clock::now() < deadline) {
    uint8_t byte{};
    const auto n = ::read(fd_, &byte, 1);
    if (n == 1) {
      result.push_back(byte);
      // Reject noise, echoes, extra IDs and malformed frames. Never silently use old feedback.
      if (result.size() == 1 || result.size() == 2) require(byte == 255, "Bad STS header/noisy bus");
      if (result.size() == 3) require(byte == id, "Unexpected STS response ID");
      if (result.size() == 4) require(byte == length+2, "Unexpected STS response length");
      if (result.size() == static_cast<size_t>(length+6)) {
        unsigned sum=0;
        for (size_t i=2; i<result.size(); ++i) sum+=result[i];
        require((sum & 255) == 255, "STS response checksum failed");
        require(result[4] == 0, "Servo "+std::to_string(id)+" fault bits "+std::to_string(result[4]));
        return Bytes(result.begin()+5, result.end()-1);
      }
    } else {
      require(n >= 0 || errno == EAGAIN || errno == EINTR, "Serial read failed");
      pollfd p{fd_, POLLIN, 0}; ::poll(&p, 1, 1);
      require(!(p.revents & (POLLHUP | POLLERR | POLLNVAL)), "Serial adapter disconnected");
    }
  }
  throw std::runtime_error("Servo "+std::to_string(id)+" response deadline missed");
}
Bytes Bus::read(int id, int address, int length) {
  require(id >= 1 && id <= 253 && address >= 0 && address <= 255 && length >= 1 && length <= 64,
          "Invalid STS read");
  // Late replies after a failed read require a new session, not an automatic retry.
  send(packet(id, 2, {static_cast<uint8_t>(address), static_cast<uint8_t>(length)}));
  return response(id, length);
}
void Bus::goals(const std::vector<Joint> & joints, const std::vector<int> & targets) {
  require(joints.size() == 5 && targets.size() == 5, "Expected five STS targets");
  Bytes values{41, 7};
  for (size_t i=0; i<joints.size(); ++i) {
    const auto & j=joints[i]; j.radians(targets[i]);
    const int speed=j.speed();
    require(speed >= 1 && speed <= 32767, "Zero native speed is forbidden");
    // Same RAM block as FTServo SyncWritePosEx: acceleration, position, time=0, speed.
    values.insert(values.end(), {static_cast<uint8_t>(j.id), 1,
      static_cast<uint8_t>(targets[i]), static_cast<uint8_t>(targets[i] >> 8), 0, 0,
      static_cast<uint8_t>(speed), static_cast<uint8_t>(speed >> 8)});
  }
  send(packet(254, 0x83, values));
}
void Bus::torque_on(int id) {
  require(id >= 1 && id <= 253, "Invalid torque ID");
  send(packet(id, 3, {40, 1}));
  response(id, 0);
  require(read(id, 40, 1)[0] == 1, "Torque enable readback failed");
}
}  // namespace joint_effort_hardware
