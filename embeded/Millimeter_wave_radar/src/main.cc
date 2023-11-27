// can_reader.cpp
#include <iostream>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iomanip>

int main() {
    // 创建 SocketCAN 套接字
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s == -1) {
        perror("Socket creation error");
        return 1;
    }

    // 设置 CAN 接口
    struct sockaddr_can addr;
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 将套接字与 CAN 接口绑定
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        perror("Bind error");
        close(s);
        return 1;
    }

    // 读取 CAN 数据
    struct can_frame frame;
    while (true) {
        ssize_t nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("Read error");
            close(s);
            return 1;
        }

        // 处理接收到的 CAN 帧数据
        std::cout << "Received CAN Frame:" << std::hex << std::setfill('0') << std::uppercase;
        std::cout << "MsgID: " << frame.can_id << " data length: " << static_cast<int>(frame.can_dlc) << " data: ";
        for (int i = 0; i < frame.can_dlc; ++i) {
            std::cout << std::setw(2) << static_cast<int>(frame.data[i]) << " ";
        }
        std::cout << std::dec << std::endl;
    }

    // 关闭套接字
    close(s);

    return 0;
}
