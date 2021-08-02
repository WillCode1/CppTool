//on_exit()注册一个函数，在进程退出的时候调用他
#include <stdio.h>
#include <stdlib.h>
void func(int  x, void *arg){
    char *p=(char *)arg;
    printf("%d\t%s\n",x,p);
    return;
}
int main(void){
    on_exit(func,"tarena");
    exit(1);
    printf("这行打印不出来。。\n");
}
