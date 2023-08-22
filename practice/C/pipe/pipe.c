//使用匿名管道实现 "ls | wc -l"
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
//#include <errno.h>

void sys_err(const char* str) {
    perror(str);
    exit(1);
}

int main(int argc, char* argv[]) {
    int fd[2];
    int ret = pipe(fd);
    if (ret != 0) {
        sys_err("pipe error");
    }
    pid_t pid = fork();
    if (pid == 0) {
        close(fd[1]);
        dup2(fd[0], STDIN_FILENO);
        execlp("wc", "wc", "-l", NULL);
    } else if (pid > 0) {
        close(fd[0]);
        dup2(fd[1], STDOUT_FILENO);
        execlp("ls", "ls", NULL);
        //write(fd[1], "hello world\n", 13);
        close(fd[1]);
    }
}