#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "linklist.h"

void Test01() {
    struct LinkNode* head = InitLinkList();
    TraverseLinkList(head);
    //InsertNode(head, 2, 100);
    DeleteNode(head, 2);
    TraverseLinkList(head);
}

int main() {
    Test01();
    return 0;
}