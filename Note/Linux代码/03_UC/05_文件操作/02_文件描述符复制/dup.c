/* dup,dup2函数复制文件描述符
 * 通过复制文件描述符,可以实现文件流重定向redirection,输出重定向
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

int main(){
    int fd = open("a.txt", O_RDWR|O_CREAT);
    if(-1 == fd){
        perror("open");
        return -1;
    }
    //结果是先文件,后屏幕
    //1.备份fd2表示显示器输出
    int fd2 = dup(STDOUT_FILENO);
    //2.标准输出表示文件a.txt
    dup2(fd, STDOUT_FILENO);
    write(STDOUT_FILENO, "Zimmer\n", 7);
    //3.标准输出恢复正常
    dup2(fd2, STDOUT_FILENO);
    write(STDOUT_FILENO, "Zimmer\n", 7);
    //4.关闭所有文件描述符
    close(fd);
    close(fd2);
    return 0;
}
