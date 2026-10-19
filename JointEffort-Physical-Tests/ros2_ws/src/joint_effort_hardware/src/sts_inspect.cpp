#include "joint_effort_hardware/sts_bus.hpp"
#include <iostream>
#include <set>
#include <stdexcept>
using namespace joint_effort_hardware;

int main(int argc, char ** argv) {
  if (argc<3) {
    std::cerr << "Read only: sts_inspect /dev/serial/by-id/ADAPTER ID [ID ...]\n"
                 "Use one physically isolated motor if IDs are not unique. No writes are sent.\n";
    return 2;
  }
  try {
    std::set<int> ids;
    for (int i=2; i<argc; ++i) {
      size_t end{}; const std::string text(argv[i]); const int id=std::stoi(text,&end);
      if (end!=text.size() || id<1 || id>253 || !ids.insert(id).second)
        throw std::runtime_error("Give distinct IDs in 1..253");
    }
    Bus bus; bus.open(argv[1],1000000);
    for (const int id:ids) {
      const auto identity=bus.read(id,3,3);
      const auto data=bus.read(id,33,31);
      std::cout << "{\"id\":" << id << ",\"reported_id\":" << static_cast<int>(identity[2])
        << ",\"model\":" << word(identity,0) << ",\"mode\":" << static_cast<int>(data[0])
        << ",\"torque\":" << static_cast<int>(data[7])
        << ",\"counts\":" << signed_word(word(data,23))
        << ",\"voltage_v\":" << .1*data[29]
        << ",\"temperature_c\":" << static_cast<int>(data[30]) << "}\n";
    }
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Read-only inspection failed: " << e.what() << '\n'; return 1;
  }
}
