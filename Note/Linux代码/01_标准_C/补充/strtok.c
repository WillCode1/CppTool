//字符串分割函数strtok
//实际上是把第二个字符串中的字符都换成'\0'
#include <string.h>
#include <stdio.h>

int main(int argc, char* argv[]){
    if(argc < 3){
        char str[] = "minwei@tedu.cn";
        //分隔符集合
        char sep[] = "@.";
        char* p = strtok(str, sep);
        // minwei\0tedu.cn
        // p       ^
        printf("%s\n", p); // minwei
        p = strtok(NULL, sep);
        // tedu\0cn
        // p     ^
        printf("%s\n", p); // tedu
        p = strtok(NULL, sep);
        // cn
        // p ^
        printf("%s\n", p); // cn
        p = strtok(NULL, sep);
        // p = NULL
        printf("%p\n", p);
    }
    else{
        char* str = argv[1];
        char* sep = argv[2];
        char* sub = NULL;
        for(sub = strtok(str, sep); sub; sub = strtok(NULL, sep))
            printf("%s\n", sub);
    }
    return 0;
}
