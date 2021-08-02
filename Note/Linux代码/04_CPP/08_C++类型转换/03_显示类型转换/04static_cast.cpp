//静态类型转换
#include <iostream>
using namespace std;

int main(void)
{
    int* pi = NULL;
    void* pv = pi;
    pi = static_cast<int*>(pv);
    
    //C++风格强制转换,不合理，使用危险
    char *pc = int(pi);
    //静态转换会检查合理性，不合理报错
    //pc = static_cast<char*>(pi);
    return 0;
}
