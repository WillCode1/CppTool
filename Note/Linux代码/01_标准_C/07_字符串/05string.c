/*
   字符串函数演示,
   fgets(buf, sizeof(str), stdin);
   加上缓冲区处理
   */
#include <stdio.h>
#include <string.h>
int main() {
    char buf[10] = {0};
    printf("请输入一个字符串：");
    //scanf("%s", buf);
    fgets(buf, 10, stdin);
    if (strlen(buf) == 9 && buf[8] != '\n') {
        scanf("%*[^\n]");
        scanf("%*c");
    }
    printf("%s\n", buf);
    printf("请输入一个字符串：");
    //scanf("%s", buf);
    fgets(buf, 10, stdin);
    printf("%s\n", buf);
    return 0;
}
