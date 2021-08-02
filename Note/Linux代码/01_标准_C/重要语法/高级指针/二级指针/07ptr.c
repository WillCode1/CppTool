/*
   二级指针演示
   一个*抵消一个p
   */
#include <stdio.h>
int main() {
    int num = 0;
    int *p_num = &num;
    int **pp_num = &p_num;
    **pp_num = 10;
    printf("num是%d\n", num);
    *pp_num = NULL;
    printf("p_num是%p\n", p_num);
    return 0;
}
