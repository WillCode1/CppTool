/*
   文件操作演示
   fwrite函数写入文件
   */
#include <stdio.h>
int main() {
    int arr[] = {1,2,3,4,5};
    int size = 0;
    FILE *p_file = fopen("a.bin", "wb");
    if(!p_file){
        return 0;
    }
    size = fwrite(arr, sizeof(int), 5, p_file);
    printf("写入的存储区个数是%d\n", size);
    fclose(p_file);
    p_file = NULL;
    return 0;
}
