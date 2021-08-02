/*变长数组练习
 * 编写程序从键盘得到多个考试成绩，找到其中最大成绩和最小成绩并计算总成绩把他们都显示在屏幕上,成绩数由用户给出
 */
#include<stdio.h>
int main(){
    int qty = 0, min = 100, max = 0, num = 0, sum = 0;
    printf("请输入成绩个数！\n");
    scanf("%d",&qty);
    int arr[qty];   //变长数组不可初始化
    for(num = 0; num < qty; num++){
        printf("请输入一个成绩！");
        scanf("%d", &arr[num]);
        if(arr[num] < min){
            min = arr[num];
        }
        if(arr[num] > max){
            max = arr[num];
        }
        sum += arr[num];
    }
    printf("最大成绩为%d，最小成绩为%d，总成绩为%d\n", max, min, sum);
    return 0;
}
