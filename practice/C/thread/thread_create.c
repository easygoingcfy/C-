#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

void sys_err(char* str) {
  perror(str);
  exit(1);
}

// subthread callback
void* start_routine(void* arg) { 
    printf("tfn: pid = %d, pthread_id = %lu\n",  getpid(), pthread_self());
    return NULL; 
    }

int main(int argc, char* argv[]) {
  pthread_t tid;
  int ret;
  ret = pthread_create(&tid, NULL, start_routine, NULL);
  if (ret != 0) {
    fprintf(stderr, "pthread_create err:%s\n", strerror(ret));
  }
    printf("main: pid = %d, pthread_id = %lu\n",  getpid(), pthread_self());
    sleep(1);
    return 0;
}