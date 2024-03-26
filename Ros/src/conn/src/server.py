import sys
import socket

def server(ip, port):
    """
    UDP服务器端
    :return:
    """
    SERVER_IP = ip  # 监听IP地址
    SERVER_PORT = port  # 监听端口号

    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 绑定socket到指定的IP地址和端口号
    sock.bind((SERVER_IP, SERVER_PORT))

    print(f"UDP服务器已启动，监听{SERVER_IP}:{SERVER_PORT}")

    while True:
        # 接收数据
        data, addr = sock.recvfrom(1024)  # 1024是接收缓冲区的大小
        print(f"收到来自{addr}的消息: {data.decode('utf-8')}")

        # 发送同样的消息作为回复
        sock.sendto(data, addr)

    # 关闭socket
    sock.close()


def client():
    """
    UDP客户端
    :return:
    """
    SERVER_IP = "127.0.0.1"  # 服务器IP地址
    SERVER_PORT = 8889  # 服务器端口号
    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        # 发送数据
        data = input("请输入要发送的数据：")
        sock.sendto(bytes(data, encoding="utf-8"), (SERVER_IP, SERVER_PORT))
        # 接收数据
        recv_data, addr = sock.recvfrom(1024)
        print(f"收到来自{addr}的消息: {recv_data.decode('utf-8')}")
    # 关闭socket
    sock.close()

if __name__ == '__main__':
    if (sys.argv[1] == "server"):
        server("127.0.0.1", 8889)
    elif (sys.argv[1] == "client"):
        client()
    else:
        print("Usage: python server.py [server|client]")