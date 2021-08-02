/*
   枚举类型演示,本质就是整数类型,用于逻辑区分
   */
#include <stdio.h>
int main() {
    enum {CHUN, XIA = 5, QIU, DONG};
    printf("CHUN是%d\n", CHUN);
    printf("QIU是%d\n", QIU);
    return 0;
}
