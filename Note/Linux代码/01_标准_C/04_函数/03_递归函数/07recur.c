/*
   µİ¹éº¯ÊıÑİÊ¾,´òÓ¡1~n
   */
#include <stdio.h>
void print(int num) {
	if (num == 1) {
		printf("1 ");
		return ;
	}
	print(num - 1);
	printf("%d ", num);
}
int main() {
	print(5);
	printf("\n");
	return 0;
}
