/* 函数练习，编写函数打印以上内容，每行内容用一个函数调用语句打印
 * 1 x 9 = 9
 * 2 x 8 = 16
 * 3 x 7 = 21
 * 4 x 6 = 24
 * 5 x 5 = 25
 */
#include<stdio.h>

void func(int num){
    printf("%d x %d = %d\n", num, 10 - num, num * (10 - num));
}

int main(){
    int num = 0;
    for(num = 1; num <= 5; num++){
        func(num);
    }
    return 0;
}
