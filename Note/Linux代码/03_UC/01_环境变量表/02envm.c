//main函数参数里的第三个参数省略的是,环境表的首地址
#include <stdio.h>

int main(int argc, char *argv[], char *env[]){
    int i = 0;
    //不能使用env++
    for(; *(env+i); i++){
        printf("%s\n", *(env+i));
    }
}
