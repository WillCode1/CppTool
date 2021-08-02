/*
   变长数组演示
   */
#include <stdio.h>
int main() {
	int num = 0;
	printf("请输入一个整数：");
	scanf("%d", &num);
	int arr[num];    //变长数组
	printf("sizeof(arr)是%d\n", sizeof(arr));
	return 0;
}

