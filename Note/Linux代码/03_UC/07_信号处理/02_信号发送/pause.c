//pause函数使用
//等待一个信号，并处理
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

void handler(int signum){}

int main(void){
    signal(SIGALRM,handler);
    while(1){
        alarm(3);
        pause();
        printf("hello ni hao ...\n");
    }
    return 0;
}
