/*
   宏操作符演示
   #, ##
   */
#include <stdio.h>
#define    STR(n)   #n
#define    VAR(n)   i_##n
int main() {
    int num = 0;
    //int i_num = 0;
    int VAR(num) = 0;
    printf("%s\n", STR(2 + 4));
    return 0;
}
