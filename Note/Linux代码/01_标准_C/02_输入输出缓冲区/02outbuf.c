/*
   输出缓冲区演示
   fflush(stdout);
   */
#include <stdio.h>
int main() {
	printf("1");
	//printf("1\n");    //换行字符前的内容会显示在屏幕上
	fflush(stdout);
	while (1) {         //没有死循环时，主函数结束后会打印
	//    printf("1");  //当输出缓冲区被充满时，里面的内容会打印在屏幕上
	}
	return 0;
}
