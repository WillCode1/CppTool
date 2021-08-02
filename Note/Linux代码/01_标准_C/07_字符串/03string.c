/*
   字符串函数演示
   sprintf,sscanf
   对字符串写入，读取数据
   */
#include <stdio.h>
int main() {
    char buf[20] = {0};
    int num = 0;
    float fnum = 0.0f;
    char ch = 0;
    //printf("%d %g %c\n", 34, 5.4f, 'y');
    sprintf(buf, "%d %g %c", 34, 5.4f, 'y');
    printf("%s\n", buf);
    //scanf("%d %g %c", &num, &fnum, &ch);
    sscanf("98 3.1 p", "%d %g %c", &num, &fnum, &ch);
    printf("%c %d %g\n", ch, num, fnum);
    return 0;
}
