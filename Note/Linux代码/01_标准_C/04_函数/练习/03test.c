/*编写函数生成一张彩票，在主函数里把彩票打印在屏幕上
 */
#include<stdio.h>
#include<stdlib.h>
#include<time.h>

void lottery(int arr[], int);

int main(){
    int num = 0;
    int arr[7] = {0};
    lottery(arr, 7);
    printf("彩票号码是：");
    for(num = 0; num < 7; num++){
        printf("%d ", arr[num]);
    }
    printf("\n");
    return 0;
}

void lottery(int arr[], int size){
    int num = 0, tmp = 0;
    srand(time(0));
    do{
        tmp = rand() % 36 + 1;
        for(num = 0; num < size; num++){
            if(arr[num] == tmp){
                break;  //有相同数字，不用在比较了
            }
            else if(!arr[num]){
                arr[num] = tmp;
                break;  //成功放入一个数字，在比较会导致重复
            }
        }
    }while(!arr[size - 1]);
}
