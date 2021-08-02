//系统调用write函数演示
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

int main(){
    //1.打开文件
    int fd = open("a.txt", O_WRONLY|O_CREAT|O_EXCL, 0664);
    if(fd == -1){
        perror("open");
        return -1;
    }
    printf("打开文件成功\n");
    //2.写入文件
    int res = write(fd, "hello", strlen("hello"));
    if(-1 == res){
        perror("write"); 
        return -1;
    }
    printf("写入文件成功，写入的数据大小是：%d\n", res);
    //3.关闭文件
    res = close(fd);
    if(res == -1){
        perror("close");
        return -1;
    }
    printf("关闭文件成功\n");
    return 0;
}
