//使用有名管道实现进程间的通信
//用mkfifo命令创建有名管道文件
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main(void){
    int fd = open("a.pipe",O_WRONLY);
    if(-1 == fd){
        perror("open"),exit(-1);
    }
    //2.写入数据到文件
    int i = 0;
    for(i = 0; i < 100; i++){
        int res = write(fd,&i,sizeof(int));
        if(-1 == res){
            perror("write"),exit(-1);
        }
        //每隔0.1秒写一次
        usleep(100000);
    }
    close(fd);
    //练习：vi 01pipeB.c 每隔0.1秒读取管道中数据并且打印出来
    return 0;
}
