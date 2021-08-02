//基于TCP通信模型的客户端
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <string.h>
#include <stdlib.h>
int main(void){
    char buf[128];
    struct sockaddr_in serv;
    //创建socket
    int s_fd=socket(AF_INET,SOCK_STREAM,0);
    if(s_fd==-1){
        perror("socket");
        return 1;
    }
    serv.sin_family=AF_INET;
    serv.sin_port=htons(7777);
    //将字符串格式的IP地址转换为结构体类型
    inet_pton(AF_INET,"127.0.0.1",&serv.sin_addr);
    int ret=connect(s_fd,(struct sockaddr *)&serv,sizeof(serv));
    if(ret==-1){
        perror("connect");
        return 3;
    }
    //给服务器发送字符串
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
