/*
   变量生命周期演示
   */
#include <stdio.h>
void func(void) {
    static int num = 100;   //静态局部变量，生命周期变长，作用域没变
    printf("func.num是%d\n", num);
    num = 10;
}
void func1(void) {
    int num = 100;  //同名局部变量num,存储区不同
    func();
    printf("func1.num是%d\n", num);
}
int main() {
    func();
    func1();
    return 0;
}
