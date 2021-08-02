//编写程序实现拷贝文件
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
int main(int argc, char *argv[]){
    int r_res = 0, w_res = 0;
    char *tmp = 0;
    char buf[100] = {0};
    if(argc < 3){
        printf("请输入文件路径\n");
        return 1;
    }
    else if(!strcmp(argv[1],argv[2])){
        printf("新文件路径不要与原文件相同\n");
        return 2;
    }
    int fd1 = open(argv[1], O_RDONLY);
    if(-1 == fd1){
        printf("原文件打开失败\n");
        return 3;
    }
    int fd2 = open(argv[2], O_WRONLY|O_CREAT|O_TRUNC, 0664);
    if(-1 == fd2){
        printf("新文件名已存在\n");
        return 4;
    }
    while(r_res = read(fd1, buf, sizeof(buf)) > 0){
        //需要确保读进缓存的东西都写出后,再进行下一次
        tmp = buf;
        while(w_res = write(fd2, tmp, strlen(buf)) > 0){
            tmp += w_res;
            r_res -= w_res;
            if(!r_res)  break;
        }
        bzero(buf, 100);
    }
    close(fd1);
    close(fd2);
    return 0;
}
