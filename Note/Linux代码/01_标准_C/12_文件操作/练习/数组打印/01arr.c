/* 文件操作练习，左边的写入文件，文件中读出右边(二维数组)
 * 11111    55555
 * 22222    44444
 * 33333    33333
 * 44444    22222
 * 55555    11111
 */
#include<stdio.h>

int main(){
    int row = 0;
    int arr[][5] = {1,1,1,1,1,2,2,2,2,2,
        3,3,3,3,3,4,4,4,4,4,5,5,5,5,5};
    FILE *p_file = fopen("arr.bin", "wb");
    if(p_file){
        for(row = 0; row < 5; row++){
            //fwrite(arr, sizeof(int), 25, p_file);
            fwrite(arr[row], sizeof(int), sizeof(arr[0])/sizeof(int), p_file);
            }
        fclose(p_file);
        p_file = NULL;
        }
    return 0;
}
