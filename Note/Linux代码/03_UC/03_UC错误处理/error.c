//错误处理函数
//fopen,printf,strerror和全局变量errno
#include <stdio.h>
#include <errno.h>
#include <string.h>

int main(){
    FILE *p_file = fopen("etc/pwd", "w");
    if(!p_file){
        perror("fopen");
        //printf("errno=%d\n", errno);
        printf("%s\n", strerror(errno));
        printf("%m\n");
        return -1;
    }
    fclose(p_file);
    return 0;
}
