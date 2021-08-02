/*
   条件编译演示(多选一)
   */
#include <stdio.h>
int main() {
#if   defined(GONGCHANG)
    printf("80%%\n");
#elif !defined(GONGCHANG) && !defined(JINGPIN)
    printf("100%%\n");
#else
    printf("120%%\n");
#endif
    return 0;
}
