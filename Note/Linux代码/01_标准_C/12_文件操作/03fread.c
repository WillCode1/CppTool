/*
   文件操作演示
   fread函数读取文件
   */
#include<stdio.h>
int main(){
    int arr[5] = {0}, size = 0, num = 0;
    FILE *p_file = fopen("a.bin", "rb");
    if(p_file){
        size = fread(arr, sizeof(int), 5, p_file);
        printf("一共获得%d个整数\n", size);
        fclose(p_file);
        p_file = NULL;
    }
    printf("数组内容为：");
    for(num = 0; num < size; num++){
        printf("%d ", arr[num]);
    }
    printf("\n");
    return 0;
}
