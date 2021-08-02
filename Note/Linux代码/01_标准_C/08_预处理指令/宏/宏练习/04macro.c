/*
 * 宏练习，编写宏实现字符大小写转换
 * 所有(公式内)变量加括号，整个公式加括号
 */
#include<stdio.h>
#define     CHANGE(p)   ((p) >= 'A' && (p) <= 'Z' ? (p) - 'A' + 'a' : (p) - 'a' + 'A')

int main() {
    char ch = 0;
    printf("请输入一个字符：");
    scanf("%c", &ch);
    printf("转换后的字符是：%c\n", CHANGE(ch));
    return 0;
}
