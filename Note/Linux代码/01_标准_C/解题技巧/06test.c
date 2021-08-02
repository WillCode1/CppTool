/* 有n人围成圈子，顺序排号，从第一个人开始报数(从1到3)，凡报到3的人退出圈子
 * 求最后剩下的人是原来的第几个人,请用指针
 */
#include<stdio.h>

int main(){
    int size = 0, num = 0, cnt = 1, quit = 0;
    //int arr[size];    //变长数组声明，本来在这，程序无限循环
    printf("请输入初始总人数：");
    scanf("%d", &size);
    int arr[size];	//变长数组最好在数组元素个数赋值后在声明
    for(num = 0; num < size; num++){
        arr[num] = num + 1;
    }
    while(1){
        for(num = 0; num < size; num++){
            if(!arr[num]){
                continue;
            }
            else if(cnt % 3 == 0){
                arr[num] = 0;
                quit++;
            }
            cnt++;
        }
        if(quit == size - 1){
            break;
        }
    }
    for(num = 0; num < size; num++){
        if(arr[num]){
            printf("剩下的人的原来编号是%d\n", arr[num]);
        }
    }
    return 0;
}
