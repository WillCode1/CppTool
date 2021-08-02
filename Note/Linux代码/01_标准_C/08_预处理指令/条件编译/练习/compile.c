/* 编写点菜程序(这个真没有)
 * 1代表龙虾(有，没有)
 * 2代表小鸡炖蘑菇(没有，真没有)
 * 程序需要根据点的菜给出相应的提示
 * 程序需要编译两个不同的版本
 */
#include<stdio.h>
int main(){
    int order = 0;
    printf("请输入点菜的序号：");
    scanf("%d", &order);
#ifdef  ZHAOBENSHAN
    if(order == 2){
        printf("真没有\n");
    }
    else{
        printf("没有\n");
    }
#else
    if(order == 1){
        printf("有\n");
    }
    else{
        printf("没有\n");
    }
#endif
    return 0;
}
