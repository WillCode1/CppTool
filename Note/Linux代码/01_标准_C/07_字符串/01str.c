/*
   字符串演示
   */
#include <stdio.h>
int main() {
    char str[] = "abc";
    printf("%p\n", "abc");
    printf("%p\n", "ab""c");
    printf("%c\n", *"abc");     //一般情况下不会这么写
    printf("%d\n", *("abc" + 3));
    //*"abc" = 'z';    错误
    printf("sizeof(str)是%d\n", sizeof(str));
    printf("%s\n", "abc");
    printf("%s\n", str);
    return 0;
}
