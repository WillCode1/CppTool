//alarm函数的使用
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>

void fa(int signo)
{
    printf("捕获到了信号%d\n",signo);
    alarm(1);
}

int main(void)
{
    signal(SIGALRM,fa);
    int res = alarm(10);
    printf("res = %d\n",res);//0
    sleep(2);
    //修改为3秒后发送SIGALRM信号
    res = alarm(3);
    printf("res = %d\n",res); //8
#if 0
    //设置为0秒后,表示取消之前的闹钟
    res = alarm(0);
    printf("res = %d\n",res); //3
#endif
    while(1);
    return 0;
}
