/* 二维数组练习，填入5 x 5，并打印，不可用初始化的方法初始0以上的数
 * 11112
 * 40002
 * 40002
 * 40002
 * 43333
 */
#include<stdio.h>
int main(){
    int row = 0, col = 0;
    int arr[5][5] = {0};
    int cur_row = 0, cur_col = 0;   //当前位置
    int step_row = 0, step_col = 1; //走的方向
    int tmp_row = 0, tmp_col = 0;   //走完后的位置
    //一种方法，较简单
/*    for(row = 0; row < 5; row++){
        for(col = 0; col < 5; col++){
            if(!row && col < 4){
                arr[row][col] = 1;
            }
            else if(col == 4 && row < 4){
                arr[row][col] = 2;
            }
            else if(row == 4 && col > 0){
                arr[row][col] = 3;
            }
            else if(!col && row > 0){
                arr[row][col] = 4;
            }
        }
    }*/
    //第二种方法,一共放4个数字，每个放4次
    for (row = 1;row <= 4;row++) {
	for (col = 0;col <= 3;col++) {
	    arr[cur_row][cur_col] = row;  //在当前所在位置放数字
	    tmp_row = cur_row + step_row;
    	    tmp_col = cur_col + step_col;
            if (tmp_row < 0 || tmp_row > 4 || tmp_col < 0 || tmp_col > 4) {
		if (!step_row && step_col == 1) {
                    //向右改向下
    		    step_row = 1;
                    step_col = 0;
                }
                else if (step_row == 1 && !step_col) {
                    //向下改向左
    		    step_row = 0;
                    step_col = -1;
                }
		else if (!step_row && step_col == -1) {
    		    //向左改向上
                    step_row = -1;
    		    step_col = 0;
                }
                else {
    		    //向上改向右
                    step_row = 0;
    		    step_col = 1;
                }
                tmp_row = cur_row + step_row;
                tmp_col = cur_col + step_col;
            }
		//用下一步的位置替换当前位置
                cur_row = tmp_row;
	        cur_col = tmp_col;
        }
    }
    //输出二维数组
    for(row = 0; row < 5; row++){
        for(col = 0; col < 5; col++){
            printf("%d", arr[row][col]);
        }
        printf("\n");
    }
    return 0;
}
