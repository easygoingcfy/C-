#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
  struct Student stu;
  int fd = open("test.map", O_RDONLY);
  ftruncate(fd, sizeof(stu));
  struct Student* p = mmap(NULL, sizeof(stu), PROT_READ, MAP_SHARED, fd, 0);
  if (p == MAP_FAILED) {
    sys_err("mmap error");
  }
  close(fd);
  while (1) {
    printf("id = %d, name = %s, age = %d\n", p->id, p->name, p->age);
    sleep(2);
  }
  munmap(p, sizeof(stu));
  return 0;
}
