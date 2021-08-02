//基于TCP通信模型的并发服务器
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <strings.h>
#include <ctype.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <string.h>
#include <sys/wait.h>
#include <stdlib.h>
int main(void){
    struct sockaddr_in serv,cli;
    char buf[128]={0}, str[20]={0};
    //创建socket
    int sockfd=socket(AF_INET,SOCK_STREAM,0);
    if(sockfd==-1){
        perror("socket"),exit(-1);
    }
    //初始化服务器的地址和端口信息
    serv.sin_family= AF_INET;
    serv.sin_port=htons(7777);
    //INADDR_ANY 这台机器上的所有网络地址
    serv.sin_addr.s_addr=htonl(INADDR_ANY);
    //1.解决重新运行时地址被占用的问题
    //int reuseaddr = 1;
    //setsockopt(sockfd,SOL_SOCKET,SO_REUSEADDR,&reuseaddr,sizeof(reuseaddr));
    //绑定sockfd和服务器地址
    int ret=bind(sockfd,(struct sockaddr *)&serv,sizeof(serv));
    if(ret==-1){
        perror("bind"),exit(-1);
    }
    //2.监听客户的连接(数量限制?)
    listen(sockfd,5);
    printf("listening ...\n");
    while(1){
        int len=sizeof(cli); 
        int connfd=accept(sockfd,(struct sockaddr *)&cli,&len);
        //char *ip = inet_ntoa(cli.sin_addr);
        char *ip = (char *)inet_ntop(AF_INET,&cli.sin_addr,str,20);
        printf("连接上的客户端IP：%s\n",ip);
        //3.回收僵尸子进程的资源
        waitpid(-1,NULL,WNOHANG);
        //生成一个子进程来响应客户端
        pid_t pid=fork();
        if(pid<0){
            perror("fork"),exit(-1);
        }
        if(pid>0){  //父进程,负责等待下一个客户端的连接
            close(connfd);
        }
        else {  //子进程,负责和客户端
            close(sockfd);
            while(1){	
                bzero(buf,128);
                //获取客户端传送过来的数据
                int r=recv(connfd,buf,128,0);
                //业务处理
                if(strcmp(buf,"exit\n")==0){
                    printf("下线的客户端IP：%s\n",ip);
                    break;
                }
                int i = 0;
                for(i=0;i<r;i++){
                    buf[i]=toupper(buf[i]);
                }
                //响应客户端
                send(connfd,buf,r,0);
            }
            close(connfd);
            exit(0);
        }
    }
    close(sockfd);
    return 0;
}
