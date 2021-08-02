//重析构(double free)
#include <iostream>

int main(){
    int *p = new int;
    delete p;
    //delete p;   //重析构,核心转储
    
    //delete空指针安全
    p = NULL;
    delete p;

    //delete野指针,后果未定义,可能会核心转储
    int *p1;
    delete p1;
    return 0;
}
