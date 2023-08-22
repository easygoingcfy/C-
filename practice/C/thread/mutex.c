#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// create mutex
pthread_mutex_t mutex;


void* tfn(void* arg) {
  srand(time(NULL));
  while (1) {
    int ret = pthread_mutex_lock(&mutex);
    if (ret != 0) {
      printf("lock error: %d\n", ret);
      continue;
    }
    printf("hello ");
    sleep(rand() % 3);
    printf("world\n");
    pthread_mutex_unlock(&mutex);
    sleep(rand() % 1);
  }
  return NULL;
}

int main(int argc, char* argv[]) {
  pthread_t tid;
  srand(time(NULL));
  //init
  pthread_mutex_init(&mutex, NULL);
  pthread_create(&tid, NULL, tfn, NULL);
  while (1) {
    int ret = pthread_mutex_lock(&mutex);
    if (ret != 0) {
      printf("lock error: %d\n", ret);
      continue;
    }
    printf("HELLO ");
    sleep(rand() % 3);
    printf("WORLD\n");
    pthread_mutex_unlock(&mutex);
    sleep(rand() % 1);
  }
  return 0;
}