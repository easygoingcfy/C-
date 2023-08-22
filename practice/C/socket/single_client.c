#include <stdio.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

void sys_err(char *str) {
    perror(str);
    exit(1);
}

int main() {
    //create socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd == -1) {
        perror("socket");
        exit(1);
    } 

    //connect
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(8989); 
    inet_pton(AF_INET, "10.0.3.15", &addr.sin_addr.s_addr);
    int ret = connect(sockfd, (struct sockaddr*)&addr, sizeof(addr));
    if (ret == -1) {
        sys_err("bind error");
    }
     
    //communication
    int num = 0;
    while (1) {
        char buf[1024];
        sprintf(buf, "hello world, %d...", num++);
        send(sockfd, buf, strlen(buf) + 1, 0);

        memset(buf, 0, 1024);
        int len = recv(sockfd, buf, sizeof(buf), 0);
        if (len == 0) {
            printf("server close connection");
            break;
        } else if (len > 0) {
            printf("client recv: %s\n", buf);
        } else {
            perror("client recv err");
            break;
        }
        sleep(1);
    }

    //close
    close(sockfd);

    return 0;
}