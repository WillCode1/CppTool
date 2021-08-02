//基于UDP通信模型的客户端
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <strings.h>
typedef struct sockaddr SA;
typedef struct sockaddr_in SAI;

int main(void){
    SAI serv;
    char buf[128]="hello tarena...";
    //创建socket
    int s_fd=socket(AF_INET,SOCK_DGRAM,0);
    //初始化服务器的地址和端口
    serv.sin_family=AF_INET;
    serv.sin_port=htons(7779);
    inet_pton(AF_INET,"127.0.0.1",&serv.sin_addr);
    //数据的发送
    int len=sizeof(SA);
    sendto(s_fd,buf,strlen(buf)+1,0,(SA *)&serv,len);
    //等待服务器的响应
    bzero(buf,128);
    int r=recvfrom(s_fd,buf,128,0,NULL,NULL);
    //write(1,buf,r);
    printf("服务器发来信息：%s\n",buf);
    return 0;
}
