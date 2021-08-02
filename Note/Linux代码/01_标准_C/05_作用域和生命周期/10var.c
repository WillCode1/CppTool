/*
   变量作用域演示
   */
#include <stdio.h>
int num1;           //全局变量
void func(void) {
	//printf("num是%d\n", num);
	printf("num1是%d\n", num1);
}
int main() {
	int num = 0;    //局部变量
	int num1 = 10;
	printf("num是%d\n", num);
	printf("num1是%d\n", num1);
	func();
	return 0;
}
