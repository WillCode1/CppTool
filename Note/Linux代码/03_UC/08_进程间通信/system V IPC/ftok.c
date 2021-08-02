//通过路径和项目ID来获取key值(键值)
//注意：使用相同路径和项目ID会生成相同的key值

#include <stdio.h>
#include <sys/types.h>
#include <sys/ipc.h>

int main(void){
    key_t key;
    //key=ftok(".",23);
    key=ftok(".",22);
    printf("key=%d\n",key);
    return 0;
}
