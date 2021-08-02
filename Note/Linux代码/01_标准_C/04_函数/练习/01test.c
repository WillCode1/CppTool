/* 函数练习，鸡兔同笼
 * 编写函数输入头脚，输出结果
 */
#include<stdio.h>
int rabbit(int heads, int legs){
    int num = 0;
    for(num = 0; num <= heads; num++){
        if(4 * num + 2 * (heads - num) == legs){
            return num;
        }
    }
}

int main(){
    int heads = 0, legs = 0, result = 0;
    printf("请输入头和脚的总数：");
    scanf("%d%d", &heads, &legs);
    result = rabbit(heads, legs);
    printf("兔子有%d只，鸡有%d只。\n", result, heads - result);
    return 0;
}
