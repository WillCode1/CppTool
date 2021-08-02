/* 字符串练习，编写程序把字符串中所有考试成绩和计算出来并显示在屏幕上
 * 10，20，30，40，50
 */
#include<stdio.h>
#include<stdlib.h>
#include<string.h>

int main(){
    char *p_grade = "10,20,30,40,50";
    int sum = 0;
    while(1){
        sum += atoi(p_grade);
        p_grade = strstr(p_grade, ",");
        if(!p_grade){
            break;
        }
        p_grade++;
    }
    printf("考试成绩总和为：%d\n", sum);
    return 0;
}
