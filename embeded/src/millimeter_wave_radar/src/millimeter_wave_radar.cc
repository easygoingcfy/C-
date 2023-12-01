#include <ros/ros.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <condition_variable>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "millimeter_wave_radar/MmwObject.h"
#include "millimeter_wave_radar/MmwObjects.h"

using Object = millimeter_wave_radar::MmwObject;
using Objects = millimeter_wave_radar::MmwObjects;

// struct Object {
//   uint8_t id;             // 目标id
//   float dist_long;        // 目标纵向距离
//   float dist_lat;         // 目标横向距离
//   float vrel_long;        // 目标纵向速度
//   float vrel_lat;         // 目标横向速度
//   uint8_t sector_number;  // 扇区编号
// 
//   friend std::ostream& operator<<(std::ostream& os, const Object& obj) {
//     os << "Object {" << std::endl
//        << std::left << std::setw(20) << "id: " << static_cast<int>(obj.id) << std::endl
//        << std::setw(20) << "dist_long: " << obj.dist_long << std::endl
//        << std::setw(20) << "dist_lat: " << obj.dist_lat << std::endl
//        << std::setw(20) << "vrel_long: " << obj.vrel_long << std::endl
//        << std::setw(20) << "vrel_lat: " << obj.vrel_lat << std::endl
//        << std::setw(20) << "sector_number: " << static_cast<int>(obj.sector_number) << " }" << std::endl
//        << std::right;
//     return os;
//   }
// };

class Radar {
 public:
  Radar() : ifname_("can0") {
    objects_pub_ = gnh_.advertise<millimeter_wave_radar::MmwObjects>("objects", 10);
  }

  /**
   * @brief 获取检测到的目标信息
   *
   * @return std::vector<Object>*
   */
  std::vector<Object>* getObjects() { return &objects_; }

  /**
   * @brief 主流程，负责从can接口读取消息并解析
   *
   */
  void run() {
    std::thread reader_thread(std::bind(&Radar::can_reader, this, ifname_));

    while (true) {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return !can_queue_.empty(); });

      // 从队列中取出 CAN 帧并处理
      can_frame frame = can_queue_.front();
      can_queue_.pop();

      // 处理 CAN 帧
      process_can_frame(frame);
    }

    reader_thread.join();
  }

 private:
  void can_reader(const char* ifname) {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
      perror("Error creating socket");
      return;
    }

    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname);
    ioctl(sock, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      perror("Error binding socket");
      close(sock);
      return;
    }

    while (true) {
      can_frame frame;
      ssize_t nbytes = read(sock, &frame, sizeof(frame));
      if (nbytes < 0) {
        perror("Error reading from socket");
        break;
      } else if (nbytes == sizeof(frame)) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          can_queue_.push(frame);
        }
        cv_.notify_one();
      }
    }

    close(sock);
  }

  void process_can_frame(const can_frame& frame) {
    // 处理收到的 CAN 帧
    std::cout << "Received CAN frame: ID=0x" << std::hex << frame.can_id << ", DLC=" << static_cast<int>(frame.can_dlc)
              << ", Data=";
    for (int i = 0; i < frame.can_dlc; ++i) {
      std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(frame.data[i]) << " ";
    }
    std::cout << std::dec << std::setfill(' ') << "\n";
    if ((uint32_t)frame.can_id == 0x60a) {
      auto head_info = parseObjectListStatus(frame);
      std::cout << "n: " << head_info[0] << std::endl;
      std::cout << "meas count: " << head_info[1] << std::endl;
      std::cout << "interface version: " << head_info[2] << std::endl;
    } else if ((uint32_t)frame.can_id == 0x60b) {
      parseObjectGeneralInformation(frame);
    }
  }

  /**
   * @brief  解析目标列表头信息(0x60a)
   *
   * @param frame
   * @return std::vector<int>
   */
  std::vector<int> parseObjectListStatus(const can_frame& frame) {
    // 先清空objects_
    publishObjects();
    std::vector<Object>().swap(objects_);

    int objects_number = static_cast<int>(frame.data[0]);
    int objects_meas_count = (static_cast<int>(frame.data[1]) << 8) + static_cast<int>(frame.data[2]);
    int objects_interface_version = static_cast<int>(frame.data[3]) >> 4;
    return std::vector<int>{objects_number, objects_meas_count, objects_interface_version};
  }

  /**
   * @brief 解析目标的距离和速度信息, 存放到objects_中
   *
   * @param frame
   */
  void parseObjectGeneralInformation(const can_frame& frame) {
    Object obj;
    obj.id = static_cast<int>(frame.data[0]);

    float min, res;  // Res: 分辨率  Min:偏移量
    int output;
    // dist longitudinal
    min = -500, res = 0.2;
    output = (static_cast<int>(frame.data[1]) << 5) + (static_cast<int>(frame.data[2]) >> 3);
    obj.dist_long = output * res + min;

    // dist lateral
    min = -204.6, res = 0.2;
    output = (static_cast<int>(frame.data[2] & 7) << 8) + static_cast<int>(frame.data[3]);
    obj.dist_lat = output * res + min;

    // vrel longitudinal
    min = -128.0, res = 0.25;
    output = static_cast<int>(frame.data[4] << 2) + static_cast<int>(frame.data[5] >> 6);
    obj.vrel_long = output * res + min;

    // vrel lateral
    min = -64, res = 0.25;
    output = (static_cast<int>(frame.data[5] & 0x3f) << 3) + static_cast<int>(frame.data[6] >> 5);
    obj.vrel_lat = output * res + min;

    // sector number
    obj.sector_number = static_cast<int>((frame.data[6] >> 3) & 0x03);

    std::cout << obj;

    objects_.emplace_back(obj);
  }


  /**
   * @brief 发布雷达信息话题
   * 
   */
  void publishObjects() {
    Objects obj;
    obj.objects = objects_;
    objects_pub_.publish(obj);
  }

  const char* ifname_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<Object> objects_;
  std::queue<can_frame> can_queue_;

  // ros
  ros::NodeHandle gnh_;
  ros::Publisher objects_pub_;
};
