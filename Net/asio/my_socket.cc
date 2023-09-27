#include "pinling_protocol.cc"

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int main() {
    // 创建套接字
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Error creating socket" << std::endl;
        return 1;
    }

    // 设置服务器的地址信息
    struct sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(2000);  // 服务器端口
    inet_pton(AF_INET, "192.168.2.119", &(serverAddress.sin_addr));

    // 连接到服务器
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1) {
        std::cerr << "Error connecting to server" << std::endl;
        close(clientSocket);
        return 1;
    }
    std::cout << "connect success" << std::endl;

    // 连接成功后，你可以在这里发送和接收数据
    // 例如，使用 write() 发送数据，使用 read() 接收数据
    CombinationControlA cmd = get_cmd();
    write(clientSocket, (const void*)&cmd, sizeof(cmd) + 1);
    char buf[1024];
    std::cout << "write done" << std::endl;
    read(clientSocket, buf, 1);
    std::cout << buf << std::endl;

    // 关闭套接字
    close(clientSocket);

    return 0;
}
