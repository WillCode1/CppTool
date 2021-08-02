//使用raise函数发送信号
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

void handler(int signo)
{
    printf("捕获到了信号%d\n",signo);
}

int main(void)
{
    //设置信号2进行自定义处理
    signal(2,handler);
    int res = sleep(5);
    if(0 == res)
    {
        printf("总算美美地睡了个好觉\n");
    }
    else
    {
        printf("睡眠被打断，还有%d秒没有来得及睡\n",res);
    }
    //使用raise函数发送信号2
    raise(2);
    while(1);
    return 0;
}
