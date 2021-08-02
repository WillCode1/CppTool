/* 文件操作练习，读取文件
 */
#include<stdio.h>

int main(){
    int row = 0, col = 0, arr[5][5] = {0};
    FILE *p_file = fopen("arr.bin", "rb");
    if(p_file){
        for(row = 4; row >= 0; row--){
            fread(arr[row], sizeof(int), 5, p_file);
        
        }
        for(row = 0; row < 5; row++){
            for(col = 0; col < 5; col++){
                printf("%d", arr[row][col]);
            }
            printf("\n");
        }
        fclose(p_file);
        p_file = NULL;
    }
    return 0;
}
