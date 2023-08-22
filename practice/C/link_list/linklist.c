#include "linklist.h"

struct LinkNode* InitLinkList() {
  struct LinkNode* head = malloc(sizeof(struct LinkNode));
  if (head == NULL) {
    return NULL;
  }
  head->num = -1;
  head->next = NULL;

  struct LinkNode* tail = head;
  int val = -1;
  while (1) {
    printf("请插入数据，输入-1代表插入结束\n");
    scanf("%d", &val);
    if (val == -1) {
      break;
    }
    struct LinkNode* cur = malloc(sizeof(struct LinkNode));
    cur->num = val;
    cur->next = NULL;
    tail->next = cur;
    tail = tail->next;
  }
  return head;
}

// traverse linklist
void TraverseLinkList(struct LinkNode* head) {
  if (head == NULL) {
    return;
  }
  struct LinkNode* cur = head->next;
  while (cur != NULL) {
    printf("%d\n", cur->num);
    cur = cur->next;
  }
}

// insert node
void InsertNode(struct LinkNode* head, int old_val, int new_val) {
  if (head == NULL) {
    return;
  }
  struct LinkNode* cur = head;
  while (cur->next != NULL) {
    if (cur->next->num == old_val) {
        struct LinkNode* new_node = malloc(sizeof(struct LinkNode));
        new_node->num = new_val;
        new_node->next = cur->next;
        cur->next = new_node;
        break;
    }
    cur = cur->next;
  }
}

//delete node
void DeleteNode(struct LinkNode* head, int val) {
    if (head == NULL) {
        return;
    }
    struct LinkNode* cur = head;
    while (cur->next != NULL) {
        if (cur->next->num == val) {
          struct LinkNode* temp = cur->next;
          cur->next = cur->next->next;
          free(temp);
          temp = NULL;
          break;
        }
        cur = cur->next;
    }
}

void ClearLinkList(struct LinkNode* head) {
    if (head == NULL) {
        return;
    }
    struct LinkNode* cur = head->next;
    while (cur != NULL) {
        struct LinkNode* next = cur->next;
        free(cur);
        cur = next;
    }
    head->next = NULL;
}

void DestroyLinkLisk(struct LinkNode* head) {
    if (head == NULL) {
        return;
    }
    ClearLinkList(head);
    free(head);
    head = NULL;
}