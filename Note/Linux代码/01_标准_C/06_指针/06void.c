/*
   无类型指针演示,void
   */
#include <stdio.h>
int main() {
    char ch = 'a';
    int num = 45;
    float fnum = 3.2f;
    void *p_v = NULL;
    p_v = &ch;
    printf("%c\n", *(char *)p_v);
    p_v = &num;
    printf("%d\n", *(int *)p_v);
    p_v = &fnum;
    printf("%g\n", *(float *)p_v);
    //普通指针也可以强制转换
    int *p_num = &num;
    num += 20;
    printf("%c\n", *(char *)p_num);
    return 0;
}
