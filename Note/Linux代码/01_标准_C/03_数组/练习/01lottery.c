//彩票练习，7个数字彩票,1~36,数字不重复，用户买彩票后，显示买中的个数
#include<stdio.h>
#include<stdlib.h>
#include<time.h>

int main (void){
    int num = 0, num1 = 0, cnt = 0;
    int lottery[7] = {0}, tmp = 0;  //中奖彩票
    int arr[7] = {0};         //用户买的彩票
    srand(time(0));
    do{
        tmp = rand() % 36 + 1;
        for(num = 0; num < 7; num++){
            if(tmp == lottery[num])
            {
                break;
            }
            if(!lottery[num]){
                lottery[num] = tmp;
                break;
            }
        }
    }while(!lottery[6]);
    for(num = 0; num < 7; num++){
        printf("%d ",lottery[num]);
    }
    printf("\n");

    for(num = 0; num < 7; num++){
        printf("请输入一个彩票数字：");
        scanf("%d ", &arr[num]);
    }

    //检查买中几个数字
    for(num = 0; num < 7; num++){
        for(num1 = 0; num1 < 7; num1++){
            if(lottery[num] == arr[num1]){
                cnt++;
                break;
            }
        }        
    }
    printf("用户买中%d了个数字！\n", cnt);
    return 0;
}
