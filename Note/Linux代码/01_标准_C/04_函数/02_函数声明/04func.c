/*
   函数显示声明演示
   */
#include <stdio.h>
double add(double, double);	//没有这句话会报错

int main() {
    int num = add(3, 8);
	printf("num是%d\n", num);
	return 0;
}
double add(double num, double num1) {
	return num + num1;
}
