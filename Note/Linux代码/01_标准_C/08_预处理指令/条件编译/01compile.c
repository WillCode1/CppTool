/*
   条件编译演示(二选一)
   */
#include <stdio.h>
int main() {
//#ifdef       YI     //cc -DYI 01compile.c,通过执行第一条
#ifndef       ER    //编译时不定义宏执行此条
    printf("1\n");
#else
    printf("2\n");
#endif
    return 0;
}
