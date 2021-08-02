//递归打印目录中的所有内容
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>

void print(char* path)
{
    //1.打开目录
    DIR* dir = opendir(path);
    if(NULL == dir)
    {
        return;
    }
    //2.不断地读取目录中的内容
    struct dirent* ent;
    while(ent = readdir(dir))
    {
        //3.如果读取到的是目录，则先打印，再递归打印子目录
        if(4 == ent->d_type)
        {
            printf("[%s]\n",ent->d_name);
            //特殊处理 .  ..
            if(!strcmp(ent->d_name,".") || !strcmp(ent->d_name,".."))
            {
                continue;
            }
            //拼接新的子目录路径
            char buf[100] = {0};
            sprintf(buf,"%s/%s",path,ent->d_name);
            //递归打印子目录
            print(buf);
        }
        //4.如果读取到的是文件，则直接打印
        if(8 == ent->d_type)
        {
            printf("%s\n",ent->d_name);
        }
    }
}

int main(int argc, char *argv[])
{   
    if(argc < 2){
        printf("请输入目录\n");
        return -1;
    }
    print(argv[1]);
    return 0;
}
