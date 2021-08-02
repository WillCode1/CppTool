//信号的异步
//可重入函数和不可重入函数
//发送信号2,以验证
#include <stdio.h>
#include <signal.h>
void handle(int signum){
    static int count = 1;
    //int count = 1;
    int ret;
    ret = count;
    sleep(1);
    count = ++ret;
    printf("%d\n",count);
}

int main(void){
    signal(SIGINT,handle);
    while(1){
        handle(2);
    }
    return 0;
}
