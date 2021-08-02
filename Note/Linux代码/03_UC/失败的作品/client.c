//基于TCP通信模型的客户端
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/sem.h>
int main(void){
    char buf[128],start[] = "客户端已登陆！";
    struct sockaddr_in serv;
    //创建socket
    int s_fd=socket(AF_INET,SOCK_STREAM,0);
    if(s_fd==-1){
        perror("socket"),exit(-1);
    }
    serv.sin_family=AF_INET;
    serv.sin_port=htons(7777);
    //将字符串格式的IP地址转换为结构体类型
    inet_pton(AF_INET,"127.0.0.1",&serv.sin_addr);
    int ret=connect(s_fd,(struct sockaddr *)&serv,sizeof(serv));
    if(ret==-1){
        perror("connect"),exit(-1);
    }
    //创建并初始化一个信号量集控制服务器链接数
    key_t key = ftok(".",20);
    if(-1 == key){
        perror("ftok"),exit(-1);
    }
    int semid = semget(key,1,IPC_CREAT|0664);
    if(-1 == semid){
        perror("semget"),exit(-1);
    }
    ret = semctl(semid,0,GETVAL);
    if(ret == -1){
        perror("semctl"),exit(-1);
    }
    struct sembuf op1 = {0,-1,IPC_NOWAIT};
    struct sembuf op2 = {0,-1,SEM_UNDO};
    semop(semid,&op1,1);
    //是否排队等待
    send(s_fd,&ret,4,0);
    if(0 == ret){
        printf("服务器繁忙，请等待！\n");
        //等待占用一个服务器链接
        ret = semop(semid,&op2,1);
        if(-1 == ret){
            perror("semop"),exit(-1);
        }
        send(s_fd,&ret,4,0);
    }
    printf("%s\n",start);
    //给服务器发送字符串
    write(1,buf,ret);
    while(fgets(buf,128,stdin)!=NULL){
        if(strlen(buf)==127&&buf[126]!='\n'){
            scanf("%*[^\n]");
            scanf("%*c");
        }
        send(s_fd,buf,strlen(buf)+1,0);
        if(strcmp(buf,"exit\n")==0){
            printf("本客户端已下线！\n");
            break;
        }
        bzero(buf,128);
        //从服务器得到响应
        int r=recv(s_fd,buf,128,0);
        write(1,buf,r);
    }
    close(s_fd);
    return 0;
}
