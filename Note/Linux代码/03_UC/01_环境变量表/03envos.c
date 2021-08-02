/* 修改环境变量的函数
 * getenv,putenv,setenv,unsetenv,clearenv
 */
#include <stdio.h>
#include <stdlib.h>

//环境变量只在当前进程起作用
int main(){
    printf("HOME=%s\n", getenv("HOME"));
    putenv("NAME=Zimmer");
    //setenv("NAME", "Stark", 1);
    setenv("NAME", "Stark", 0);
    printf("NAME=%s\n", getenv("NAME"));
    unsetenv("NAME");
    printf("NAME=%s\n", getenv("NAME"));
    if(!clearenv()){
        printf("成功清空整个环境表\n");
    }
    return 0;
}
