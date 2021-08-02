/*
   字符串函数演示
   strlen,strcat,strcmp,strcpy,memset,strstr
   */
#include <stdio.h>
#include <string.h>
int main() {
    char str[20] = "abcxyz";
    printf("strlen(str)是%d\n", strlen(str));
    printf("sizeof(str)是%d\n", sizeof(str));
    printf("%s\n", strcat(str, "def"));
    printf("%s\n", str);
    printf("%s\n", strncat(str, "iop", 2));
    printf("%s\n", str);
    printf("比较结果是%d\n", strcmp("abc", "abd"));
    printf("比较结果是%d\n", strncmp("abc", "abd", 2));
    printf("复制结果是%s\n", strcpy(str, "jiklk"));
    printf("复制结果是%s\n", str);
    printf("%s\n", strncpy(str, "abc", 2));
    printf("%s\n", str);
    memset(str, 'h', 15);
    printf("%s\n", str);
    //strstr函数返回找到结果第一字符的位置
    printf("%s\n", strstr("abcdefghijklmn", "def"));
    return 0;
}
