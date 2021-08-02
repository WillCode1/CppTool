//使用共享内存实现进程间的通信
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/shm.h>

int main(void){
    //1.获取key值,使用ftok函数
    key_t key = ftok(".",150);
    if(-1 == key){
        perror("ftok"),exit(-1);
    }
    //2.获取共享内存，使用shmget函数
    int shmid = shmget(key,0,0);
    if(-1 == shmid){
        perror("shmget"),exit(-1);
    }
    //3.挂接共享内存，使用shmat函数
    char *str = (char*)shmat(shmid,NULL,0);
    if((void*)-1 == str){
        perror("shmat"),exit(-1);
    }
    printf("挂接共享内存成功\n");
    //4.使用共享内存
    printf("共享内存中的数据是：%s\n", str);
    //5.脱接共享内存，使用shmdt函数
    int res = shmdt(str);
    if(-1 == res){
        perror("shmdt"),exit(-1);
    }
    printf("脱接共享内存成功\n");
    return 0;
}
