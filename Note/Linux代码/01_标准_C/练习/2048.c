#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#define      SIZE          4
enum {UP, RIGHT, DOWN, LEFT};  //代表四个方向的枚举值
//初始化地图，把每个位置初始化成0
void init_map(int map[][SIZE]/*代表棋盘的数组*/, int size/*棋盘的行数*/) {
    int row = 0, col = 0;
    for (row = 0;row <= size - 1;row++){
        for (col = 0;col <= SIZE - 1;col++){
            map[row][col] = 0;
        }
    }
}
//显示地图，每个数字占6个位置
void show_map(int map[][SIZE]/*代表棋盘的数组*/, int size/*棋盘的行数*/) {
    int row = 0, col = 0;
    for (row = 0;row <= size - 1;row++) {
        for (col = 0;col <= SIZE - 1;col++) {
            printf("%6d", map[row][col]);
        }
        printf("\n");
    }
}
//在一个空的地方增加一个新的2
void add_num(int map[][SIZE]/*代表棋盘的数组*/, int size/*棋盘的行数*/) {
    int row = 0, col = 0, cnt = 0, pos = 0, sn = 0;
    //统计空位置的个数
    for (row = 0;row <= size - 1;row++) {
        for (col = 0;col <= SIZE - 1;col++) {
            if (!map[row][col]){
                cnt++;
            }
        }
    }
    //为每个空位置指定一个虚拟编号，
    //产生一个随机的虚拟编号，
    pos = rand() % cnt;
    sn = 0;
    //通过循环查找指定虚拟编号对应的位置
    for (row = 0;row <= size - 1;row++) {
        for (col = 0;col <= SIZE - 1;col++) {
            if (!map[row][col]) {
                if (sn == pos) {
                    //在虚拟编号对应的位置上
                    //放置一个2
                    map[row][col] = 2;
                    return ;
                }
                else {
                    sn++;
                }
            }
        }
    }
}
//把所有位置上的数字向某个方向滑动一次
void slip(int map[][SIZE]/*代表棋盘的数组*/, int size/*棋盘的行数*/) {
    int direction = 0/*用户选择的滑动方向*/,
    num = 0/*代表方向的循环变量*/,
    row = 0/*行号循环变量*/,
    col = 0/*列号循环变量*/;
    //每个滑动方向有四个起点，每个起点的位置需要
    //两个整数记录。三维数组用来记录四个滑动方向
    //上所有起点的位置
    int start_points[][SIZE][2] = {
        {{0, 0}, {0, 1}, {0, 2}, {0, 3}},
    {{0, size - 1}, {1, size - 1}, {2, size - 1}, {3, size - 1}},
    {{3, 0}, {3, 1}, {3, 2}, {3, 3}},
    {{0, 0}, {1, 0}, {2, 0}, {3, 0}}};
    //每个方向上格子的处理顺序使用两个整数表示
    //四个方向上不同的处理顺序用二维数组记录
    static const int directions[][2] = {{1, 0},
        {0, -1}, {-1, 0}, {0, 1}};
    //获得用户指定的滑动方向
    printf("请选择一个方向：%d代表上，%d代表右，%d代表下，%d代表左：", UP, RIGHT, DOWN, LEFT);
    scanf("%d", &direction);
    for (num = 0;num <= 3;num++){
        //每个滑动方向需要处理四组格子
        //每循环一次处理一组格子
        //找到当前组中第一个格子的位置
        row = start_points[direction][num][0];
        col = start_points[direction][num][1];
        do {
            //每循环一次处理当前组中的一个格子
            if (row < 0 || row >= size
                || col < 0 || col >= SIZE) {
                //如果要处理的格子位置不在棋盘里
                //就说明当前组里的所有格子都处理完了
                break;
            }
            if (map[row][col]) {
                //如果要处理的格子里有数字
                //就要对这个数字做移动处理
                //oldrow和oldcol记录当前格子的位置
                int oldrow = row;
                int oldcol = col;
                //tmprow和tmpcol记录滑动方向上下一个格子
                //的位置
                int tmprow = oldrow - directions[direction][0];
                int tmpcol = oldcol - directions[direction][1];
                while (tmprow >= 0 && tmprow <= size - 1
                       && tmpcol >= 0 && tmpcol <= SIZE - 1) {
                    //如果下一个格子在棋盘里就需要处理
                    if (!map[tmprow][tmpcol]) {
                        //如果下一个格子里没有数字就
                        //把当前格子里的数字直接移动过去
                        map[tmprow][tmpcol] = map[oldrow][oldcol];
                        map[oldrow][oldcol] = 0;
                    }
                    else {
                        //如果下一个格子里有数字并且和当前
                        //格子里的数字相同就需要合并
                        //如果下一个格子里的数字是这次刚刚合并
                        //后的结果则它一定是奇数，不可能和当前
                        //格子里的数字相等
                        if (map[tmprow][tmpcol] == map[oldrow][oldcol]) {
                            map[tmprow][tmpcol] <<= 1;
                            //刚合并后的数字必须是奇数，这样
                            //可以避免多次合并
                            map[tmprow][tmpcol] |= 1;
                            map[oldrow][oldcol] = 0;
                        }
                        //当前格子处理完了
                        break;
                    }
                    //为继续滑动做准备
                    oldrow = tmprow;
                    oldcol = tmpcol;
                    tmprow = oldrow - directions[direction][0];
                    tmpcol = oldcol - directions[direction][1];
                }
            }
            //为当前组中下一个格子的处理做准备
            row += directions[direction][0];
            col += directions[direction][1];
        } while (1);
    }
    //当所有合并工作完成后把数组里
    //所有的奇数恢复成偶数
    for (row = 0;row <= size - 1;row++) {
        for (col = 0;col <= SIZE - 1;col++) {
            if (map[row][col] & 1) {
                map[row][col] ^= 1;
            }
        }
    }
}
//判断棋盘是不是已经满了，如果满了则
//返回1否则返回0
int full(int map[][SIZE]/*代表棋盘的数组*/, int size/*棋盘的行数*/) {
    int row = 0, col = 0;
    for (row = 0;row <= size - 1;row++) {
        for (col = 0;col <= SIZE - 1;col++) {
            if (!map[row][col]) {
                return 0;
            }
        }
    }
    return 1;
}
int main()
{
    int map[SIZE][SIZE] = {}, choice = 1;
    srand(time(0));
    do {
        init_map(map, SIZE);  //初始化棋盘
        add_num(map, SIZE);   //增加一个数字2
        show_map(map, SIZE);  //显示开始的棋盘
        do {
            slip(map, SIZE);  //滑动一次
            if (full(map, SIZE)) {
                //如果棋盘满了则游戏结束了
                show_map(map, SIZE);
                printf("游戏结束\n");
                break;
            }
            else {
                //如果棋盘没有满就再加一个
                //数字然后显示棋盘
                add_num(map, SIZE);
                show_map(map, SIZE);
            }
        } while (1);
        printf("是否需要开始下一盘？0表示不，1表示是");
        scanf("%d", &choice);
    } while (choice);
    return 0;
}
