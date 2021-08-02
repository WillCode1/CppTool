//变长参数表
//va_list, va_start, va_arg, va_end
#include <stdarg.h>
#include <stdio.h>
void foo(int arg1, ...){
    //打印arg1
    printf("%d ", arg1);
    //1.创建变长参数表(动态内存分配)
    va_list ap;
    //2.存放所给参数后面的"..."部分
    va_start(ap, arg1);
    int argx;
    //3.
    while((argx = va_arg(ap, int)) != -1)
        printf("%d ", argx);
    printf("\n");
    //4.销毁变长参数表
    va_end(ap);
}
int main(void){
    foo(10, -1);
    foo(10, 20, -1);
    foo(10, 20, 30, -1);
    foo(10, 20, 30, 40, -1);
    foo(10, 20, 30, 40, 50, -1);
    return 0;
}
