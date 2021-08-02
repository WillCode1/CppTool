/*递归函数演示，打印1~n
 */
#include<stdio.h>
void print(int num){
    if(num == 1){
        printf("1"); 
        return;     //递归函数必须有return;
    }
    else{
        print(num - 1);
        printf("%d" ,num);
    }
}
int main(){
    int num = 0;
    printf("请输入一个数字：");
    scanf("%d", &num);
    print(num);
    printf("\n");
    return 0;
}
