//环境变量表遍历
//等同于命令env
#include <stdio.h>

int main(){
    extern char **environ;
    for(;*environ != NULL; environ++){
        printf("%s\n", *environ);
    }
    return 0;
}
