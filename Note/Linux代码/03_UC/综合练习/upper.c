//编译生成upper可执行文件
#include <stdio.h>
#include <ctype.h>
int main(void){
    int c;
    while((c=getchar())!=EOF){
        printf("%c",toupper(c));
    }
    return 0;
}
