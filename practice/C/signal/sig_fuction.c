#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void sys_err(char* str) {
  perror(str);
  exit(1);
}

void PrintPending(sigset_t* set) {
  for (int i = 1; i < 32; ++i) {
    if (sigismember(set, i)) {
      putchar('1');
    } else {
      putchar('0');
    }
  }
  putchar('\n');
}

int main() {
  sigset_t set, oldset, pendset;
  int ret = sigemptyset(&set);
  if (ret == -1) {
    sys_err("emptyset err");
  }
  ret = sigaddset(&set, SIGINT);
  if (ret == -1) {
    sys_err("emptyset err");
  }
  ret = sigprocmask(SIG_BLOCK, &set, &oldset);
  while (1) {
    sigpending(&pendset);
    PrintPending(&pendset);
    sleep(1);
  }

  return 0;
}