/*递归函数练习，1~n求和
 */
#include<stdio.h>

int sum(int num){
    if(num == 1){
        return 1;
    }
    return sum(num - 1) + num;
}

int main(){
    int num = 0;
    scanf("%d", &num);
    printf("1~n的和为：%d\n", sum(num));
    return 0;
}
