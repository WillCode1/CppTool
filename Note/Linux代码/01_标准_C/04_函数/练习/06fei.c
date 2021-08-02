/*斐波那数列练习，1，1，2，3，5，8，13，21...
 *      对应编号，0，1，2，3，4，5，6...
 */
#include<stdio.h>

int fei(int num){
    if(num == 1 || !num){
        return 1;   
    }
    return fei(num - 1) + fei(num - 2);
}

int main(){
    int num = 0;
    printf("请输入一个编号：");
    scanf("%d", &num);
    printf("结果是：%d\n", fei(num));
    return 0;
}
