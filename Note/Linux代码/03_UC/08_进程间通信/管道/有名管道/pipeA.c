//程序从管道里读取
//mkfifo函数创建管道
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <strings.h>
#include <unistd.h>
#include <fcntl.h>
#define FNAME "/home/tarena/1608/uc/m.pipe"

int main(void){
    int fd;
    char buf[128];
    //创建管道
    mkfifo(FNAME,0664);
    //以只读方式打开管道文件
    fd=open(FNAME,O_RDONLY);
    if(fd<0){
            perror("open");
            return 1;
    }
    bzero(buf,128);
    //从管道文件里读取内容
    read(fd,buf,127);
    printf("%s\n",buf);
    close(fd);
    return 0;
}
