#include <stdio.h>
#include <string.h>
#include <stdlib.h>

struct LinkNode {
  int value;
  struct LinkNode* next;
};

void DyLink() {
  struct LinkNode* node1 = malloc(sizeof(struct LinkNode));
  struct LinkNode* node2 = malloc(sizeof(struct LinkNode));
  struct LinkNode* node3 = malloc(sizeof(struct LinkNode));
  struct LinkNode* node4 = malloc(sizeof(struct LinkNode));
  struct LinkNode* node5 = malloc(sizeof(struct LinkNode));
  node1->value = 10;
  node1->next = node2;
  node2->value = 20;
  node2->next = node3;
  node3->value = 30;
  node3->next = node4;
  node4->value = 40;
  node4->next = node5;
  node5->value = 50;
  node5->next = NULL;

  struct LinkNode* cur = node1;
  while (cur != NULL) {
    printf("%d\n", cur->value);
    cur = cur->next;
  }
}

void test01() {
  FILE* f_write = fopen("./test1.txt", "w+");
  if (f_write == NULL) {
    return;
  }
  char buf[] = "hello world";
  for (int i = 0; i < strlen(buf); ++i) {
    fputc(buf[i], f_write);
  }
  fclose(f_write);
}

int main() {
  DyLink();
  return 0;
}