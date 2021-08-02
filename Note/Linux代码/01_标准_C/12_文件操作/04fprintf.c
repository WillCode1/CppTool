/*
   文本文件操作演示
   fprintf函数写入文本文件
   */
#include<stdio.h>
#include<string.h>

int main(){
    char buf[10] = {0};
    int arr[] = {10,20,30,40,50}, num = 0;
    //FILE *p_file = fopen("a.txt", "wb");
    FILE *p_file = fopen("a.txt", "w");
    if(p_file){
        for(num = 0; num < sizeof(arr)/sizeof(int); num++){
            /*sprintf(buf, "%d ", arr[num]);
            fwrite(buf, sizeof(char), strlen(buf), p_file);*/
            fprintf(p_file, "%d ", arr[num]);
        }
        fclose(p_file);
        p_file = NULL;
    }
    return 0;
}
