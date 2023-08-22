#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
#include <pthread.h>

void sys_err(char* str) {
  perror(str);
  exit(1);
}

void* callback(void* arg) {
  while (1) {
    char buf[1024];
    memset(buf, 0, sizeof(buf));
    arg = (int*)arg;
    int ret = recv(arg, buf, strlen(buf) + 1, 0);
    if (ret <= 0) {
      sys_err("send error");
    }
    printf("recv: %s\n", buf);
    sleep(1);
  }
}

int main() {
  int lfd = socket(AF_INET, SOCK_STREAM, 0);
  if (lfd == -1) {
    sys_err("socket");
  }

  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(8989);
  addr.sin_addr.s_addr = INADDR_ANY;
  int ret = bind(lfd, &addr, sizeof(addr));
  if (ret == -1) {
    sys_err("bind");
  }

  ret = listen(lfd, 128);
  if (ret == -1) {
    sys_err("listen");
  }

  struct sockaddr_in cliaddr;
  socklen_t addrlen = sizeof(cliaddr);
  while (1) {
    int cfd = accept(lfd, &cliaddr, &addrlen);
    pthread_t tid;
    pthread_create(&tid, NULL, callback, cfd);
    pthread_detach(tid);
  }

  return 0;
}