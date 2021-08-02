/* 扫雷练习,10x10地图，随机放10个地雷，
 * 分10行显示在地图上，地雷显示X，不是显示周围地雷数，没有显示O
 */
#include<stdio.h>
#include<stdlib.h>
#include<time.h>
int main(){
    int cnt = 0, row = 0, col = 0, num = 0;
    int map[10][10] = {0};
    //周围8个格子坐标
    int delta[][2] = {-1,-1,-1,0,-1,1,0,-1,0,1,1,-1,1,0,1,1};
    //放置地雷
    srand(time(0));
    do{
        row = rand() % 10;
        col = rand() % 10;
        if(map[row][col] != -1){
            map[row][col] = -1;
            cnt++;
        }
    }while(cnt < 10);
    //处理显示周围地雷个数
    for(row = 0; row < 10; row++){
        for(col = 0; col < 10; col++){
            //不处理有地雷的格子
            if(map[row][col] == -1){
                continue;
            }
            for(num = 0; num < 8; num++){
                int tmp_row = row + delta[num][0];
                int tmp_col = col + delta[num][1];
                //位于边或角的位置
                if(tmp_row < 0 || tmp_row > 9){
                    continue;
                }
                if(tmp_col < 0 || tmp_col > 9){
                    continue;
                }
                if(map[tmp_row][tmp_col] == -1){
                    map[row][col]++;
                }
            }
        }        
    }
    for(row = 0; row < 10; row++){
        for(col = 0; col < 10; col++){
            if(map[row][col] == -1){
                printf("X ");
            }
            else if(map[row][col] == 0){
                printf("O ");
            }
            else{
                printf("%d ",map[row][col]);
            }
        }
        printf("\n");
    }
    return 0;
}
