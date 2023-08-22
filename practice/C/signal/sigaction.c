#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

void sys_err(char* str) {
  perror(str);
  exit(1);
}

void sig_catch(int signum) {
    if (signum == SIGINT) {
        printf("catch signal %d", signum);
    }
    printf("return");
    return;
}

int main() {
    struct sigaction act, oldact;
    act.sa_handler = sig_catch;
    sigemptyset(&(act.sa_mask));
    act.sa_flags = 0;

    int ret = sigaction(SIGINT, &act, &oldact);
    if (ret == -1) {
        sys_err("sigaction err");
    }
    while(1);

    return 0;
}