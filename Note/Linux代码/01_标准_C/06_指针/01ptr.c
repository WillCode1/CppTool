/*
   指针变量演示
   */
#include <stdio.h>
int main() {
	int num = 0;
	//*只是为了声明该变量类型为指针，并没有参与后面的地址赋值过程
	//(int *)
	int *p_num = &num, *p_num1 = NULL;   //指针变量声明
	*p_num = 10;
	printf("num是%d\n", num);
	return 0;
}
