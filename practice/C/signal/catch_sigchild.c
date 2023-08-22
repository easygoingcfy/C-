#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>

void sys_err(char* str) {
  perror(str);
  exit(1);
}

void catch_signal(int signum) {
  pid_t wpid;
  int status;
  // if ((wpid = wait(NULL)) != -1) {  会产生僵尸进程
  while ((wpid = waitpid(-1, &status, 0)) != -1) {
    if (WIFEXITED(status)) {
      int ret = WEXITSTATUS(status);
      printf("catch child id: %d  ret: %d\n", wpid, ret);
    } else if (WIFSIGNALED(status)) {
        int ret = WTERMSIG(status);
        printf("abnormal exit. signum: %d\n", ret);
    }
  }
}

int main(int argc, char* argv[]) {
  int i;
  pid_t pid;
  for (i = 0; i < 15; ++i) {
    if ((pid = fork()) == 0) {
      break;
    }
  }
  if (i == 15) {
    // father;
    struct sigaction act, oldact;
    act.sa_handler = catch_signal;
    sigemptyset(&act.sa_mask);
    printf("%d", act.sa_flags);
    act.sa_flags = 0;
    int ret = sigaction(SIGCHLD, &act, &oldact);
    if (ret == -1) {
      sys_err("sigaction err");
    }
    while (1)
      ;
  } else {
    // son
    printf("I am child %d\n", getpid());
    return i;
  }
}