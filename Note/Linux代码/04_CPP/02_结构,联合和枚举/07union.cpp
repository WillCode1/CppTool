//定义联合时,可以省略union关键字
//C++中可以使用匿名联合
#include <iostream>
#include <cstdio>
using namespace std;

int main(void)
{
    union{//匿名联合
        unsigned int un;
        unsigned char uc[4];
    };
    un = 0x12345678;
    for(int i=0; i<4; i++){
        printf("%#x ",uc[i]);//0x78 0x56 0x34 0x12	
    }
    printf("\n");
    return 0;
}
