//进程中的内存区域划分
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int i1 = 10;//全局区
int i2; //BSS段
const int i3 = 40;//只读常量区

void fn(int i4) //栈区
{
    int i5 = 60;//栈区
    static int i6 = 70;//全局区
    const int i7 = 80;//栈区
    //p指向堆区  p本身在栈区
    int* p = (int*)malloc(4);
    //str指向只读常量区 str本身在栈区
    char* str = "good";
    //buf指向栈区  buf本身在栈区
    char buf[] = "good";
    printf("只读常量区：const修饰的全局变量i3 = %p\n",&i3);
    printf("只读常量区：str指向的字符串常量\"good\" = %p\n","good");
    printf("---------------------\n");
    printf("全局区：已初始化的全局变量i1 = %p\n",&i1);
    printf("全局区：已初始化的静态变量i6 = %p\n",&i6);
    printf("BSS段：未初始化的全部变量i2 = %p\n",&i2);
    printf("-------------------\n");
    printf("堆区：p指向的动态分配内存 = %p\n",p);
    printf("-------------------\n");
    printf("栈区：函数形参i4 = %p\n",&i4);
    printf("栈区：局部变量i5 = %p\n",&i5);
    printf("栈区：const修饰的局部变量i7 = %p\n",&i7);
    printf("栈区：指向动态分配内存的指针p = %p\n",&p);
    printf("栈区：指向字符串常量的指针str = %p\n",&str);
    printf("栈区：buf指向的字符数组 = %p\n",buf);
    printf("栈区：字符数组名buf本身 = %p\n",&buf);
}

int main(void)
{
    printf("进程中的内存布局以及地址：\n");
    printf("--------------------\n");
    printf("代码区：函数名fn = %p\n",fn);
    fn(10);
    return 0;
}
