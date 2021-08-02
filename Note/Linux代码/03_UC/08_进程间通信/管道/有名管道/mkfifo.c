//使用mkfifo函数创建有名管道文件
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#define FNAME "/home/tarena/Desktop/m.pipe"

int main(void){
    mkfifo(FNAME,664);
    return 0;
}
