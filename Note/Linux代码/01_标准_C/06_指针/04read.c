/*
   指针类型返回值演示
   */
#include <stdio.h>
int *read(void) {
    //不可以把非静态局部变量的地址作为返回值使用
    static int num = 0;
    printf("请输入一个数字：");
    scanf("%d", &num);
    return &num;
}
int main() {
    int *p_num = NULL;
    p_num = read();
    printf("%d\n", *p_num);
    return 0;
}
