/* 字符串练习，编写程序从键盘得到多个考试成绩(个数不超过10个)
 * 把所有考试成绩按如下格式拼凑成字符串并显示在屏幕上
 * 10，20，30，40，50
 */
#include<stdio.h>
#include<string.h>

int main(){
    int qty = 0, grade = 0, num = 0;
    char tmp[10] = {0}, buf[50] = {0};
    printf("请输入成绩个数：");
    scanf("%d", &qty);
    for(num = 0; num < qty; num++){
        printf("请输入一个成绩：");
        scanf("%d", &grade);
        sprintf(tmp, "%d, ", grade);
        strcat(buf, tmp);
    }
    buf[strlen(buf) - 1] = 0;
    printf("%s\n", buf);
    return 0;
}
