/*
   数组形式参数演示
   */
#include <stdio.h>
void print(int arr[5]) {
	int num = 0;
	for (num = 0;num <= 4;num++) {
		printf("%d ", arr[num]);
	}
	printf("\n");
}
int main() {
	int arr[] = {1, 2, 3, 4, 5};
	print(arr);
	return 0;
}
