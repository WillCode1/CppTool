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
    int res = mkdir(path2, S_IRWXU|S_IRWXG|S_IROTH|S_IXOTH);
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
    int r_size = 0, w_size = 0, tmp = 0;
    char buf[100] = {0};
    if(!strcmp(path1, path2)){
        printf("新文件路径不要与原文件相同\n");
        exit(-1);
    }
    FILE *p_src = fopen(path1, "rb");
    if(!p_src){
        printf("原始文件打开失败\n");
        exit(-1);
    }
    FILE *p_dest = fopen(path2, "wb");
    if(!p_dest){
        printf("新文件创建失败\n");
        fclose(p_src);
        p_src = NULL;
        exit(-1);
    }
    while((r_size = fread(buf, sizeof(char), 100, p_src)) > 0){
        //确保缓存里内容全部写出后，再执行下一次
        tmp = r_size;
        while((w_size = fwrite(buf, sizeof(char), tmp, p_dest)) > 0){
            tmp += w_size;
            r_size -= w_size;
            if(!r_size) break;
        }
    }
    fclose(p_dest);
    p_dest = NULL;
    fclose(p_src);
    p_src = NULL;
    return 0;
}
