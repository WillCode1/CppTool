/*
   动态分配内存演示
   */
#include <stdio.h>
#include <stdlib.h>
int main() {
    int *p_num = (int *)malloc(5 * sizeof(int));
    /*if (p_num) {
        //使用动态分配内存
        free(p_num);
        p_num = NULL;
    }*/
    if (!p_num) {
        return 0;
    }
    //释放动态分配内存
    free(p_num);
    p_num = NULL;
    return 0;
}
