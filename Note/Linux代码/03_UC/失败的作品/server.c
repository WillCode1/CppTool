//用信号量集控制服务器链接数
//基于TCP通信模型的服务器
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <strings.h>
#include <ctype.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <string.h>
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/sem.h>
typedef struct sockaddr_in SAI;
typedef struct sockaddr SA;
union semun{
    int val;
};

int main(){
    SAI serv, cli;
    char buf[128] = {0},str[20] = {0};
    int sfd = socket(AF_INET,SOCK_STREAM,0);
    if(-1 == sfd){
        perror("socket"),exit(-1);
    }
    serv.sin_family = AF_INET;
    serv.sin_port = htons(7777);
    inet_aton("127.0.0.1",&serv.sin_addr);
    //解决重新运行时地址被占用的问题
    int reuseaddr = 1;
    setsockopt(sfd,SOL_SOCKET,SO_REUSEADDR,&reuseaddr,sizeof(reuseaddr));
    int res = bind(sfd,(SA *)&serv,sizeof(serv));
    if(-1 == res){
        perror("bind"),exit(-1);
    }
    res = listen(sfd, 5);
    if(-1 == res){
        perror("listen"),exit(-1);
    }
    printf("listen...\n");
    //创建并初始化一个信号量集控制服务器链接数
    key_t key = ftok(".",20);
    if(-1 == key){
        perror("ftok"),exit(-1);
    }
    int semid = semget(key,1,IPC_CREAT|0664);
    if(-1 == semid){
        perror("semget"),exit(-1);
    }
    union semun semops;
    semops.val = 1;
    res = semctl(semid,0,SETVAL,semops);
    if(res == -1){
        perror("semctl"),exit(-1);
    }
    while(1){
        int len = sizeof(cli);
        int connfd = accept(sfd,(SA *)&cli,&len);
        if(-1 == connfd){
            perror("accept"),exit(-1);
        }
        //回收僵尸子进程
        waitpid(-1,NULL,WNOHANG);
        pid_t pid = fork();
        if(-1 == pid){
            perror("fork"),exit(-1);
        }
        else if(pid > 0){   //父进程
            close(connfd);
        }
        else {  //子进程
            close(sfd);
	    //struct sembuf op1 = {0,-1,SEM_UNDO};
            struct sembuf op2 = {0,1,SEM_UNDO};
            char *ip = (char *)inet_ntop(AF_INET,&cli.sin_addr,str,20);
            res = semctl(semid,0,GETVAL);
            if(-1 == res){
                perror("semctl"),exit(-1);
            }
	    recv(connfd,&res,4,0);
	    //send(connfd,&res,4,0);
            if(0 == res){   //排队等待
                printf("服务器链接数已满,等待中的客户端IP：%s\n",ip);
		recv(connfd,&res,4,0);
#if 0
                //等待占用一个服务器链接
                res = semop(semid,&op1,1);
             if(-1 == res){
             //       perror("semop"),exit(-1);
              //  }
	      //  res = 1;
	    send(connfd,&res,4,0);
#endif
            }
            else {  //顺利进入
                //占用一个服务器链接
            //    res = semop(semid,&op1,1);
             //   if(-1 == res){
            //        perror("semop"),exit(-1);
            //    }
            }
            printf("连接上的客户端IP：%s\n",ip);
            while(1){
		bzero(buf,128);
		res = recv(connfd,buf,128,0);
                //业务处理
                if(!strcmp(buf,"exit\n")){
                    printf("下线的客户端IP：%s\n",ip);
                    break;
                }
                int i = 0;
                for(i = 0; i < res; i++){
                    buf[i] = toupper(buf[i]);
                }
                //响应客户端
                send(connfd,buf,res,0);
            }
            close(connfd);
            //解除一个占用的链接
            res = semop(semid,&op2,1);
            if(-1 == res){
                perror("semop"),exit(-1);
            }
            exit(0);
        }
    }
    close(sfd);
    return 0;
}
