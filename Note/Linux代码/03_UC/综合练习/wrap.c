//将指定路径文件流输入重定向到屏幕上
//同时通过加载可执行文件upper将显示的字母全变成大写

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

int main(int argc,char *argv[]){
    int fd;
    fd=open(argv[1],O_RDONLY);
    if(fd<0){
        perror("open");
        return 1;
    }
    dup2(fd,STDIN_FILENO);
    execl("./upper","upper",NULL);
    close(fd);
    return 0;
}
