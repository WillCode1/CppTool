//编写程序实现cat命令
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
int main(int argc, char *argv[]){
    if(argc < 2){
        printf("请输入察看路径！\n");
        return -1;
    }
    int fd = open(argv[1], O_RDONLY);
    if(-1 == fd){
        perror("open");
        return -1;
    }
    int res = 0;
    char buf[100] = {0};
    while((res=read(fd, buf, 99))>0){
        printf("%s", buf);  //vi文件行末会自动追加'\n'
	//这种写法,当'\0'被覆盖后,会显示数组中没被覆盖的其余部分
        //strcpy(buf, "\0");	
	memset(buf,0,100);
	//bzero(buf,100);
    }
    close(fd);
    return 0;
}
