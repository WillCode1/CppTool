/* 宏练习，编写程序产生一张彩票
 * 彩票里包含多个1~MAX之间的随即数，抽取的总个数和最大数字由编译时来确定
 */
#include<stdio.h>
#include<stdlib.h>
#include<time.h>

int main(){
    int num = 0, lottery[SIZE] = {0};
    srand(time(0));
    for(num = 0; num < SIZE; num++){
        lottery[num] = rand() % MAX + 1;
    }
    for(num = 0; num < SIZE; num++){
        printf("%d ", lottery[num]);
    }
    printf("\n");
    return 0;
}
