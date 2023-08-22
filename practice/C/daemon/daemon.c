#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>


void sys_err(char* str) {
  perror(str);
  exit(1);
}

int main(int argc, char* argv[]) {
  pid_t pid;
  int ret;
  pid = fork();
  if (pid > 0) {
    exit(0);
  }
  pid = setsid();
  if (pid == -1) {
    sys_err("setpid error");
  }
  //change directory
  ret = chdir("/home/yu");
  if (ret == -1) {
    sys_err("chdir error");
  }
  //change umask
  umask(0);
  close(STDIN_FILENO);
  //file discripter redirect
  int fd = open("/dev/null", O_RDWR);
  if (fd == -1) {
    sys_err("open error");
  }
  dup2(fd, STDOUT_FILENO);
  dup2(fd, STDERR_FILENO);
  //daemon start
  while(1);
}