/*
   const关键字演示,两种位置不同作用
   */
#include <stdio.h>
int main() {
    int num = 0;
    const int *p_num = &num;
    int const *p_num2 = &num;   //这种写法与上面一样，一般不用
    int * const p_num1 = &num;
    //*p_num = 10;   错误
    p_num = NULL;
    *p_num1 = 10;
    //p_num1 = NULL;  错误
    printf("num是%d\n", num);
    //*p_num2 = 20;   错误
    p_num2 = NULL;
    printf("num是%d\n", num);
    return 0;
}
