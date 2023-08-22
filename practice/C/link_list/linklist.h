#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct LinkNode {
    int num;
    struct LinkNode* next;
};

struct LinkNode* InitLinkList();

//traverse linklist
void TraverseLinkList(struct LinkNode* head);

//insert node
void InsertNode(struct LinkNode* head, int old_val, int new_val);

//delete node
void DeleteNode(struct LinkNode* head, int val);

void ClearLinkList(struct LinkNode* head);

void DestroyLinkLisk(struct LinkNode* head);