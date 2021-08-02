//向管道里写内容
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#define FNAME "/home/tarena/1608/uc/m.pipe"

int main(void){
    int fd;
    //打开管道文件
    fd=open(FNAME,O_RDWR);
    if(fd<0){
        perror("open");
        return 1;
    }
    //向管道写内容
    write(fd,"hello tarena",13);
    close(fd);
    return 0;
}
