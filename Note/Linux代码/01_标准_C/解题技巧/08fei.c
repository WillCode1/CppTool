/*
   费氏数列练习,用数组提高计算效率
   用数组记录和返回已经计算过的数据
   */
#include <stdio.h>
//int arr[50];
int fei(int num, int arr[], int size) {
    //static int arr[50] = {0};
    if (num <= 1) {
        return 1;
    }
    if (!arr[num - 2]) {
        arr[num - 2] = fei(num - 2, arr, size);
    }
    if (!arr[num - 1]) {
        arr[num - 1] = fei(num - 1, arr, size);
    }
    return arr[num - 2] + arr[num - 1];
}
int main() {
    int num = 0;
    int arr[50] = {0};
    printf("请输入编号：");
    scanf("%d", &num);
    printf("结果是%d\n", fei(num, arr, 50));
    return 0;
}
