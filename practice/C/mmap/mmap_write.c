#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>
//#include <pthread.h>

void sys_err(const char* str) {
  perror(str);
  exit(1);
}

struct Student {
  int id;
  char name[256];
  int age;
};

int main(int argc, char* argv[]) {
  struct Student stu = {1, "caofangyu", 26};
  int fd = open("test.map", O_RDWR | O_CREAT | O_TRUNC, 0644);
  ftruncate(fd, sizeof(stu));
  struct Student* p = mmap(NULL, sizeof(stu), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (p == MAP_FAILED) {
    sys_err("mmap error");
  }
  close(fd);
  while (1) {
    memcpy(p, &stu, sizeof(stu));
    stu.id++;
    sleep(2);
  }
  munmap(p, sizeof(stu));
  return 0;
}
