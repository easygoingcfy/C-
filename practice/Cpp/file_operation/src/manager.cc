#include <iostream>
#include <string>

#include <boost/filesystem/operations.hpp>

namespace fs = boost::filesystem;

class Manager {
 public:
  Manager() {}
  int getCurSpace() { return 90; }

  int getTotalSpace() { return 100; }
};

int main() {
  fs::space_info space = fs::space("/");
  double usage = space.available * 1. / space.capacity;
  std::cout << "cur usage: " << usage << std::endl;
  if (usage > 0.9) {
    std::cout << "emergency";
  } else if (usage > 0.85) {
    std::cout << "warn";
  }

  std::cout << "Capacity: " << space.capacity / 1024 / 1024 / 1024 << " G" << std::endl;
  std::cout << "Free: " << space.free / 1024 / 1024 / 1024 << " G" << std::endl;
  std::cout << "Available: " << space.available / 1024 / 1024 / 1024 << " G" << std::endl;

  return 0;
}