#include <arpa/inet.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
#include <errno.h>

#define _XOPEN_SOURCE

void sys_err(char* str) {
  perror(str);
  exit(1);
}

void recycle(int num) {
  while (1) {
    pid_t pid = waitpid(-1, NULL, WNOHANG);
    if (pid > 0) {
      printf("回收了子进程： %d\n", pid);
    } else {
      break;
    }
  }
}

void working(int cfd) {
  while (1) {
    char buf[1024];
    memset(buf, 0, sizeof(buf));
    int len = recv(cfd, buf, sizeof(buf), 0);
    if (len == 0) {
      printf("客户端断开了连接\n");
      break;
    } else if (len < 0) {
      perror("读数据失败\n");
      break;
    }
    printf("recv: %s\n", buf);
    write(cfd, buf, len);
    sleep(1);
  }
  close(cfd);
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
  int ret = bind(lfd, (const struct sockaddr*)&addr, sizeof(addr));
  if (ret == -1) {
    sys_err("bind");
  }

  ret = listen(lfd, 128);
  if (ret == -1) {
    sys_err("listen");
  }

  // signal catch
  struct sigaction act;
  act.sa_handler = recycle;
  act.sa_flags = 0;
  sigemptyset(&act.sa_mask);
  sigaction(SIGCHLD, &act, NULL);

  struct sockaddr_in cliaddr;
  socklen_t addrlen = sizeof(cliaddr);
  while (1) {
    int cfd = accept(lfd, (struct sockaddr*)&cliaddr, &addrlen);
    if (cfd == -1) {
        if (errno == EINTR) {
            printf("程序被信号中断: %d\n", errno);
        }
      perror("建立连接失败, 尝试重新连接...\n");
      continue;
    }
    char myip[24];
    printf("客户端的IP: %s, port: %d\n",
           inet_ntop(AF_INET, &cliaddr.sin_addr.s_addr, myip, sizeof(myip)), 
           ntohs(cliaddr.sin_port));
        pid_t pid = fork();
    if (pid == 0) {
      // son
      close(lfd);
      working(cfd);
      exit(0);
    } else {
      close(cfd);
    }
  }
  return 0;
}