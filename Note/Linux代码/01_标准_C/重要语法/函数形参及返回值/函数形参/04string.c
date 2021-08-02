/* 字符串做形参，实际都是指针
 * 写一个函数，求一个字符串的长度，并输出
 */
#include<stdio.h>

int mystrlen(char *str){    //这里只可以是char *型
    char *p_str = str;
    int cnt = 0;
    while(*p_str){
        cnt++;
        p_str++;
    }
    return cnt;
}

int main(){
    char buf[50] = {0};
    int length = 0;
    printf("请输入一个字符串：");
    scanf("%s", buf);
    length = mystrlen("avdbsan");
    printf("字符串\"avdbsan\"长度为：%d\n", length);
    length = mystrlen(buf);
    printf("输入字符串长度为：%d\n", length);
    return 0;
}
