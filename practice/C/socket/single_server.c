
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

    //bind
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(8989); 
    addr.sin_addr.s_addr = INADDR_ANY;
    int ret = bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
    if (ret == -1) {
        sys_err("bind error");
    }
     
    //listen
    ret = listen(sockfd, 128);
    if (ret == -1) {
        sys_err("bind error");
    }

    //wait
    struct sockaddr_in cliaddr;
    int clilen = sizeof(cliaddr);
    int cfd = accept(sockfd, (struct sockaddr*)&cliaddr, &clilen);
    if (cfd == -1) {
        sys_err("accept err");
    }

    //communication
    while (1) {
        char buf[1024];
        memset(buf, 0, 1024);
        int len = recv(cfd, buf, sizeof(buf), 0);
        if (len == 0) {
            printf("client close connection");
            break;
        } else if (len > 0) {
            printf("recv: %s\n", buf);
        } else {
            perror("recv err");
            break;
        }

        send(cfd, buf, strlen(buf) + 1, 0);
    }

    //close
    close(cfd);
    close(sockfd);

    return 0;
}