//系统调用read函数演示
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

int main(){
    //1.打开文件
    int fd = open("a.txt", O_RDONLY);
    if(fd == -1){
        perror("open");
        return -1;
    }
    printf("打开文件成功\n");
    //2.读取文件
    char buf[100] = {0};
    int res = read(fd, buf, sizeof(buf));
    if(res == -1){
        perror("read");
        return -1;
    }
    printf("读取数据成功，读到的数据是：%s,读取的数据大小是：%d\n", buf, res);
    //3.关闭文件
    res = close(fd);
    if(fd == -1){
        perror("close");
        return -1;
    }
    printf("关闭文件成功\n");
    return 0;
}
