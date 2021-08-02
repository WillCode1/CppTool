/*
   指针演示,指针捆绑数组名称，即首地址
   */
#include <stdio.h>
int main() {
    int arr[5] = {1, 2, 3, 4, 5}, num = 0;
    int *p_num = arr;
    for (num = 0;num <= 4;num++) {
        printf("%d ", arr[num]);
        printf("%d ", p_num[num]);  //虽然作用一样，但要避免这种写法
        printf("%d ", *(p_num + num));
    }
    printf("\n");
    printf("arr是%p,arr + 3是%p\n", arr, arr + 3);
    printf("arr是%p,arr - 3是%p\n", arr, arr - 3);
    printf("&arr[3] - arr是%d\n", &arr[3] - arr);
    printf("sizeof(arr)是%d,sizeof(p_num)是%d\n", sizeof(arr), sizeof(p_num));
    printf("arr是%p,&arr是%p\n", arr, &arr);
    printf("p_num是%p,&p_num是%p\n", p_num, &p_num);
    return 0;
}
