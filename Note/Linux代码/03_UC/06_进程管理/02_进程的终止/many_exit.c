//进程的终止函数
//标C函数exit(),_Exit()，UC函数_exit()
//分别换用三个函数，体会不同

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void show(void)
{
    printf("我就是被注册的函数\n");
}

int main(void)
{
    //使用atexit函数,注册show,供进程退出时调用
    atexit(show);
    printf("main函数开始执行\n");
    //终止进程
    //exit(0);
    //_exit(0); //立即终止
    _Exit(0); //立即终止
    printf("main函数结束\n");
    return 0;
}
