/*费氏数列练习，从数组获取已经计算过的答案，来减少计算次数
 * 0,1,2,3,4,5,6,7...
 * 1,1,2,3,5,8,13,21...
 */
#include<stdio.h>
//int arr[50] = {0};    //一种方式全局变量
int fei(int num, int arr[], int size){
    //不加static会使被调用数组在函数递归过程中反复创建
    //static int arr[50] = {0}; //第二种方式静态局部变量
    if(num <= 1){
        return 1; 
    }
    //如果数组中前两项数据为0，则计算出结果
    if(!arr[num - 2]){
        arr[num - 2] = fei(num - 2, arr, size);
    }
    if(!arr[num - 1]){
        arr[num - 1] = fei(num - 1, arr, size);
    }
    return arr[num - 2] + arr[num - 1];
}

int main(){
    int num = 0;
    int arr[50] = {0};  //第三种类方式，使用主函数数组
    printf("请输入编号：");
    scanf("%d", &num);
    printf("结果是：%d\n", fei(num, arr, 50));
    return 0;
}
