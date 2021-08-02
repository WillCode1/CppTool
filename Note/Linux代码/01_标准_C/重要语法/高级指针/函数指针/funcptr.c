//函数指针数组演示
#include <stdio.h>
typedef int (*math)(int, int);

int process(math p, int a, int b){
    return p(a, b);
}

int add(int a, int b){
    return a + b;
}

int sub(int a, int b){
    return a - b;
}

int mul(int a, int b){
    return a * b;
}

int div(int a, int b){
    return a / b;
}

int main(){
    int a = 0, b = 0;
    math m[4] = {add, sub, mul, div};
    printf("请输入a和b的值：");
    scanf("%d%d", &a, &b);
    printf("%d+%d=%d\n", a, b, process(add, a, b));
    printf("%d-%d=%d\n", a, b, process(m[1], a, b));
    printf("%d*%d=%d\n", a, b, process(m[2], a, b));
    printf("%d/%d=%d\n", a, b, process(m[3], a, b));
    return 0;
}
