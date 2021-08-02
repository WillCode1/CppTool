//基于Socket的网络通信客户端
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int main(void)
{
    //1.创建socket,使用socket函数
    int sockfd = socket(AF_INET,SOCK_DGRAM,0);
    if(-1 == sockfd)
    {
        perror("socket"),exit(-1);
    }
    printf("创建socket成功\n");
    //2.准备通信地址,使用服务器地址
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(8888);
    addr.sin_addr.s_addr = inet_addr("192.168.118.130");
    //3.连接socket通信地址,connet函数
    int res = connect(sockfd,(struct sockaddr*)&addr,sizeof(addr));
    if(-1 == res)
    {
        perror("connect"),exit(-1);
    }
    printf("连接服务器成功\n");
    //4.进行通信,使用read/write函数
    res = write(sockfd,"hello",5);
    if(-1 == res)
    {
        perror("write"),exit(-1);
    }
    printf("发送数据到服务器成功,发送的数据大小是：%d\n",res);
    //5.关闭socket,使用close函数
    close(sockfd);
    return 0;
}
