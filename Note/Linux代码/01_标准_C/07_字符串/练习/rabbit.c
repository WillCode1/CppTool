/* 主函数的参数就是用字符指针数组记录多个相关字符串
 * 鸡兔同笼练习，编写程序得到头和脚的数量
 * 计算出鸡和兔子各自的数量并显示在屏幕上(不用scanf等输入语句)
 * ./a.out 40 100
 */
#include<stdio.h>
#include<stdlib.h>

int main(int argc, char *argv[]){
    int num = 0, rabbit = 0;
    int heads = atoi(argv[1]), legs = atoi(argv[2]);
    for(rabbit = 0; rabbit <= heads; rabbit++){
        if(4 * rabbit + 2 * (heads - rabbit) == legs){
            break;
        }
    }
    printf("鸡和兔子的数量分别为：%d %d\n", heads - rabbit, rabbit);
    return 0;
}
