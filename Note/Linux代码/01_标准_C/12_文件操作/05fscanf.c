/*
   文本文件操作演示
   fscanf函数读取文本文件
   */
#include<stdio.h>

int main(){
    FILE *p_file = fopen("a.txt", "r");
    int num = 0, num1 = 0;
    if(p_file){
        for(num = 0; num < 5; num++){
            fscanf(p_file, "%d", &num1);
            printf("%d ", num1);
        }
        printf("\n");
            fclose(p_file);
            p_file = NULL;
    }
    return 0;
}
