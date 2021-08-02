//编写程序实现命令cp
//存在则创建失败
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
#include <stdlib.h>

int filecopy(const char*, const char*);
int dircopy(const char*, const char*);

int main(int argc, char *argv[]){
    if(argc < 3){
        printf("请输入文件路径\n");
        exit(-1);
    }
    //判断文件类型
    struct stat st;
    int res = stat(argv[1], &st);
    if(-1 == res){
        perror("stat"), exit(-1);
    }
    //文件夹，则拷贝文件夹
    if(S_ISDIR(st.st_mode)){
        dircopy(argv[1], argv[2]);
    }
    //文件，则直接复制
    else {
        filecopy(argv[1], argv[2]);
    }
    return 0;
}
//拷贝文件夹
int dircopy(const char *path1, const char *path2){
    //创建新文件夹
    int res = mkdir(path2, 0775);
    if(-1 == res){
        perror("mkdir"), exit(-1);
    }
    //读取旧文件夹内容
    struct dirent *ent;
    DIR *dir = opendir(path1);
    if(NULL == dir){
        perror("opendir"), exit(-1);
    }
    while(ent = readdir(dir)){
        char buf1[200] = {0};
        char buf2[200] = {0};
        sprintf(buf1, "%s/%s", path1, ent->d_name);
        sprintf(buf2, "%s/%s", path2, ent->d_name);
        if(4 == ent->d_type){
            if(!strcmp(ent->d_name,".")||!strcmp(ent->d_name,"..")){
                continue;
            }
            dircopy(buf1, buf2);
        }
        else {
            filecopy(buf1, buf2);
        }
    }
    //关闭旧文件夹
    closedir(dir);
    return 0;
}
//拷贝文件
int filecopy(const char *path1, const char *path2){
    int fd1 = open(path1, O_RDONLY);
    if(!strcmp(path1, path2)){
        printf("新文件路径不要与原文件相同\n");
        exit(-1);
    }
    else if(-1 == fd1){
        printf("原文件打开失败\n");
        exit(-1);
    }
    //int fd2 = open(path2, O_WRONLY|O_CREAT|O_TRUNC, 0664);
    int fd2 = creat(path2, 0664);
    if(-1 == fd2){
        printf("新文件名已存在\n");
        exit(-1);
    }
    int r_res = 0, w_res = 0;
    char *tmp = 0;
    char buf[100] = {0};
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
