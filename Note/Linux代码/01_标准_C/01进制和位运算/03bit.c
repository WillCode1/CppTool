/*
   从键盘得到一个0到127之间的整数，把他转换成二进制并把结果显示在屏幕上,
要求先显示左边的数位后显示右边的数位
   */
#include <stdio.h>
int main() {
	int num = 0;
	unsigned char ch = 0x80;
	printf("请输入一个整数：");
	scanf("%d", &num);
        do{
            printf("%d", (num & ch) != 0);
            ch >>= 1;   //ch = ch >> 1;
        }while (ch != 1);
        printf("%d\n", (num & ch) != 0);
	return 0;
}

