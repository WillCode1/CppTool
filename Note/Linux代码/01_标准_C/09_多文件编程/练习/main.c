/* 多文件编程练习
 * 编写函数从键盘得到一个整数并把这个整数传递给调用函数
 * 在主函数里调用上面的函数，并把得到的整数显示在屏幕上
 * 要求多文件编程方式实现
 * (编译时，gcc命令中提供所有源文件名称)
 */
#include <stdio.h>
#include "read.h"

int main(){
    read();
    printf("输入的数字是：%d\n", num);
    return 0;
}
