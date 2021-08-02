//使用消息队列发送信信息
//msgget,msgsnd,msgctl函数

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

//定义消息的数据类型
typedef struct {
    long msgtype;//消息的类型
    char buf[20];//消息的内容
}Msg;

int msgid;

void fa(int signo){
    int res = msgctl(msgid,IPC_RMID,NULL);
    if(-1 == res){
        perror("msgctl"),exit(-1);
    }
    printf("删除消息队列成功\n");
    exit(0);//退出进程
}

int main(void){
    //1.获取key值
    key_t key = ftok(".",100);
    if(-1 == key){
        perror("ftok"),exit(-1);
    }
    //2.创建消息队列
    msgid = msgget(key,IPC_CREAT|IPC_EXCL|0644);
    if(-1 == msgid){
        perror("msgget"),exit(-1);
    }
    //3.发送消息
    Msg msg1,msg2;
    msg1.msgtype = 2;
    strcpy(msg1.buf,"Zimmer");
    msg2.msgtype = 1;
    strcpy(msg2.buf,"hello");
    int res = msgsnd(msgid,&msg1,sizeof(msg1.buf),0);
    if(-1 == res){
        perror("msgsnd"),exit(-1);
    }
    msgsnd(msgid,&msg2,sizeof(msg2.buf),0);
    printf("两条消息发送完毕\n");
    //4.若不再使用,使用msgctl函数删除
    printf("删除消息队列,请按ctrl+c...\n");
    signal(2,fa);
    while(1);
    return 0;
}
