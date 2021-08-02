/*指针练习，编写函数把一个数组里所有存储区的内容前后颠倒
 */
#include<stdio.h>

void reverse(int *p_arr, int size){
    int *p_start = p_arr, *p_end = p_arr + size - 1;
    int tmp = 0;
    while(p_start < p_end){
        tmp = *p_start;
        *p_start = *p_end;
        *p_end = tmp;
        p_start++;
        p_end--;
    }
}

int main(){
    int arr[] = {1,2,3,4,5,6,7}, num = 0;
    reverse(arr, 7);
    for(num = 0; num < 7; num++){
        printf("%d ", arr[num]);
    }
    printf("\n");
    return 0;
}
