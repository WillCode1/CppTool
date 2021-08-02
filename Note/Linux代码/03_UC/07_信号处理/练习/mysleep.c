//mysleep
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

void mysleep(int seconds){
	alarm(seconds);
	pause();
}

void handler(int signo){}

int main(void){
    signal(SIGALRM,handler);
    mysleep(3);
    printf("hello ni hao ...\n");
    return 0;
}
