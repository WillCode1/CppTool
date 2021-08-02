//使用消息队列接受信息
//msgget,msgsnd,msgrcv函数

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

//定义消息的数据类型
typedef struct{
    long msgtype;//消息类型
    char buf[20];//消息内容
}Msg;

int main(void)
{
    //1.获取key值
    key_t key = ftok(".",100);
    if(-1 == key){
        perror("ftok"),exit(-1);
    }
    //2.获取消息队列
    int msgid = msgget(key,0);
    if(-1 == msgid){
        perror("msgget"),exit(-1);
    }
    //3.接受消息
    Msg msg = {0};
    //接受队列中第一个消息
    int res = msgrcv(msgid,&msg,sizeof(msg.buf),0,0);
    //接受第一个类型为1消息
    //int res = msgrcv(msgid,&msg,sizeof(msg.buf),1,0);
    //接受队列中<=2的消息
    //int res = msgrcv(msgid,&msg,sizeof(msg.buf),-2,0);
    if(-1 == res){
        perror("msgrcv"),exit(-1);
    }
    printf("接受到的消息是：%ld,%s\n",msg.msgtype,msg.buf);
    return 0;
}
