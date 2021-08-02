//基于UDP通信模型的服务器
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <ctype.h>
#include <arpa/inet.h>

typedef struct sockaddr SA;
int main(void){
    char buf[128];
    struct sockaddr_in serv,cli; 
    //创建socket
    int s_fd=socket(AF_INET,SOCK_DGRAM,0);
    //初始化服务器的通讯参数
    serv.sin_family=AF_INET;
    serv.sin_port=htons(7779);
    serv.sin_addr.s_addr=htonl(INADDR_ANY);
    //绑定s_fd和服务器的地址
    bind(s_fd,(SA *)&serv,sizeof(serv));
    printf("服务器启动！\n");
    //接收客户端数据
    while(1){
        int i = 0;
        int len=sizeof(serv);
        int rcv=recvfrom(s_fd,buf,128,0,(SA *)&cli,&len);
        //来电显示,客户端IP和端口号
        char *ip=inet_ntoa(cli.sin_addr);
        int port=ntohs(cli.sin_port);
        printf("发送信息的客户端IP：%s,端口号：%d\n",ip,port);
        //数据的处理
        for(i=0;i<rcv;i++){
            buf[i]=toupper(buf[i]);
        }
        //响应客户端
        sendto(s_fd,buf,rcv,0,(SA *)&cli,sizeof(cli));
    }
    close(s_fd);
    return 0;
}
