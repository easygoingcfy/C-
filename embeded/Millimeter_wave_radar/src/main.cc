#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int main() {
    const char *canInterface = "can0";  // 替换为你的CAN接口名称

    // 打开CAN设备
    int canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (canSocket == -1) {
        perror("Error opening socket");
        return 1;
    }

    // 获取CAN接口索引
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, canInterface, IFNAMSIZ);
    if (ioctl(canSocket, SIOCGIFINDEX, &ifr) == -1) {
        perror("Error getting interface index");
        close(canSocket);
        return 1;
    }

    // 将CAN接口绑定到套接字
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(canSocket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == -1) {
        perror("Error binding socket to interface");
        close(canSocket);
        return 1;
    }

    // 读取CAN消息
    while (true) {
        struct can_frame frame;
        ssize_t bytesRead = read(canSocket, &frame, sizeof(struct can_frame));

        if (bytesRead < 0) {
            perror("Error reading from socket");
            break;
        }

        if (bytesRead == sizeof(struct can_frame)) {
            // 处理收到的CAN消息
            std::cout << "Received CAN message ID: " << std::hex << frame.can_id << std::dec << std::endl;
            // 可以继续处理frame.data等
        }
    }

    // 关闭CAN套接字
    close(canSocket);

    return 0;
}
