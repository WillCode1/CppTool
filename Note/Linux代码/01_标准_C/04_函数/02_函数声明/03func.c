/*
   函数隐式声明演示
   */
#include <stdio.h>

int main() {
    int num = add(3, 8);	//未先声明已调用,格式不同会报错
	printf("num是%d\n", num);
	return 0;
}
int add(int num, int num1) {
	return num + num1;
}
