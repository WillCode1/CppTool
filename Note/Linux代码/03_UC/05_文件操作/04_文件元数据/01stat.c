//stat函数的使用,获取文件元数据
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>

int main(void)
{
    //准备结构体变量
    struct stat st = {};
    //使用stat函数获取文件信息
    int res = stat("a.txt",&st);
    if(-1 == res)
    {
        perror("stat"),exit(-1);
    }
    //打印获取到的信息
    //mode_t => unsigned int
    //off_t => long int
    //time_t => long int
    printf("st_mode = %o,st_size = %ld,st_mtime = %ld\n",st.st_mode,st.st_size,st.st_mtime);
    printf("------------------\n");
    printf("文件的权限是：0%o\n",st.st_mode&0777);
    printf("文件的大小是：%ld\n",st.st_size);
    //使用带参宏判断文件的类型
    if(S_ISREG(st.st_mode))
    {
        printf("是普通文件\n");
    }
    if(S_ISDIR(st.st_mode))
    {
        printf("是目录文件");
    }
    printf("最后一次修改时间是：%s\n",ctime(&st.st_mtime));
    struct tm* pt = localtime(&st.st_mtime);
    printf("最后一次修改时间是：%d-%d-%d %02d:%02d:%02d\n",pt->tm_year+1900,pt->tm_mon+1,pt->tm_mday,pt->tm_hour,pt->tm_min,pt->tm_sec);
    return 0;
}
