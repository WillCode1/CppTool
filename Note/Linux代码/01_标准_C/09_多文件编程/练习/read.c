#include <stdio.h>
#include "read.h"

int num = 0;

void read(void){
    printf("请输入一个数字：");
    scanf("%d", &num);
}
