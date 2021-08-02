//遍历一个目录中的所有内容
//opendir,closedir,readdir
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>

int main(void)
{
    //1.打开目录
    DIR* dir = opendir("./");
    if(NULL == dir)
    {
        perror("opendir"),exit(-1);
    }
    //2.读取目录中所有内容
    //d_type,普通文件类型 8,目录的类型 4
    struct dirent* ent;
    while(ent = readdir(dir))
    {
        //printf("%d,%s\n",ent[0].d_type,ent[0].d_name);
        printf("%d, %s\n",(*ent).d_type,(*ent).d_name);
    }
    //3.关闭目录
    closedir(dir);
    return 0;
}
