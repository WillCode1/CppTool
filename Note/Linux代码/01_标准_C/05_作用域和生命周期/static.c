/* 静态局部变量和全局变量的区别
 * 函数执行完后，静态局部变量的存储区没释放，
 * 但是变量名已经没有意义了
 * 所以存储区可以通过指针访问，而变量名在别的函数里没有意义
 */
#include<stdio.h>

int *sum(int num1, int num2){
    static int tmp = 0;
    tmp = num1 + num2;
    return &tmp;
}

int main(){
    int num = 3, num1 = 5, *p_sum = NULL;
    p_sum = sum(3, 5);
    printf("和是：%d\n", *p_sum);
//    printf("%d\n", tmp);
    return 0;
}
