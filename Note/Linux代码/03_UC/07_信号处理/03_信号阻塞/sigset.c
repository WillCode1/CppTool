//信号集的的设置
//sigemptyset,setfillset,sigaddset,sigdelset,sigismember
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

int main(void)
{
    sigset_t set;
    printf("set = %ld\n",set);
    printf("sizeof(sigset_t) = %d\n",sizeof(sigset_t));
    //清空信号集
    sigemptyset(&set);
    printf("set = %ld\n",set);//0
    //增加信号到信号集中
    sigaddset(&set,2);
    // 0000 0010 => 1*2 = 2
    printf("set = %ld\n",set);
    sigaddset(&set,7);
    // 0100 0010 => 64+2 = 66
    printf("set = %ld\n",set);

    //从信号集中删除信号2
    sigdelset(&set,2);
    // 0100 0000 => 64
    printf("set = %ld\n",set);
    //判断信号是否存在
    if(sigismember(&set,7))
    {
        printf("信号7存在\n");
    }
    //填满信号集
    sigfillset(&set);
    printf("set = %ld\n",set);//很大的数
    return 0;
}
