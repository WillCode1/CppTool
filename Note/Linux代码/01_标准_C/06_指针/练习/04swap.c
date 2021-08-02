/*
   指针形式参数练习，编写函数把主函数里两个存储区里内容交换
   */
#include <stdio.h>
void swap(int *p_num, int *p_num1) {
	int tmp = 0;
	tmp = *p_num;
	*p_num = *p_num1;
	*p_num1 = tmp;
}
int main() {
	int num = 3, num1 = 7;
	swap(&num, &num1);
	printf("num是%d,num1是%d\n", num, num1);
	return 0;
}



