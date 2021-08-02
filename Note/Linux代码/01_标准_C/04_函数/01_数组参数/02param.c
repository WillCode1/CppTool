/*
   数组形式参数演示
   数组形式参数声明的时候可以省略存储区个数
   数组作为形式参数的时候需要另外一个整数类型形式参数,
   表示数组中包含的存储区个数
*/
#include <stdio.h>
void neg(int arr[], int size) {
	int num = 0;
	for (num = 0;num <= size - 1;num++) {
		arr[num] = 0 - arr[num];
	}
}
int main() {
	int arr[] = {1, 2, 3, 4, 5}, num = 0;
	neg(arr, 5);
	for (num = 0;num <= 4;num++) {
		printf("%d ", arr[num]);
	}
	printf("\n");
	return 0;
}
