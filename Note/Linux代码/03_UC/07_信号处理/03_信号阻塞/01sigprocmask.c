/* 信号的阻塞和未决
 * sigprocmask用于检查和改变要阻塞的信号集
 * sigpending用于获取未决信号集
 */
#include <stdio.h>
#include <signal.h>

int main(void){
    sigset_t set, oldset, pend;
    int res;
    //将集合清空
    sigemptyset(&set);
    //将2号信号添加到集合set里
    sigaddset(&set,SIGINT);
    //设置信号阻塞掩码
    sigprocmask(SIG_BLOCK,&set,&oldset);
    while(1){
        sigpending(&pend);
        res = sigismember(&pend,SIGINT);
        if(res == 0){
            printf("not pending...\n");
        }else{
            printf("SIGINT pending...\n");
            //恢复系统默认
            sigprocmask(SIG_SETMASK,&oldset,NULL);
        }
    }
    return 0;
}
