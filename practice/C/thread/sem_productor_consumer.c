//使用条件变量模拟 生产者-消费者模型
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define N 5
int queue[N];
sem_t blank_number, product_number;

void* productor(void* arg) {
  int i = 0;
  while (1) {
    sem_wait(&blank_number);
    queue[i] = rand() % 1000 + 1;
    printf("------produce---%d\n", queue[i]);
    sem_post(&product_number);
    i = (i + 1) % N;

    sleep(rand() % 3);
  }
}

void* consumer(void* arg) {
  int i = 0;
  while (1) {
    sem_wait(&product_number);
    printf("consumer %d\n", queue[i]);
    queue[i] = 0;
    sem_post(&blank_number);

    i = (i + 1) % N;
    sleep(rand() % 3);
  }
}

int main() {
  pthread_t pid, cid;
  sem_t sem;

  sem_init(&blank_number, 0, N);
  sem_init(&product_number, 0, 0);

  pthread_create(&pid, NULL, productor, NULL);
  pthread_create(&cid, NULL, consumer, NULL);

  pthread_join(pid, NULL);
  pthread_join(cid, NULL);

  sem_destroy(&blank_number);
  sem_destroy(&product_number);

  return 0;
}