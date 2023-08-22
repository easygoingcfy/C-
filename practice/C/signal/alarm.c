#include <stdio.h>
#include <unistd.h>


int main(int argc, char* argv[]) {
    alarm(1);
    int i = 0;
    for (;; ++i) {
        printf("%d\n", i);
    }
    return 0;
}