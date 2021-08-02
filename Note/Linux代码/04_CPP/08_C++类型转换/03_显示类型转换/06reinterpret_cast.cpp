//重解释类型转换
#include <iostream>
using namespace std;

int main(void)
{
    int number = 0x12345678;
    void* ps = reinterpret_cast<void*>(number);
    cout << ps << endl;

    char text[] = "0001\000123456\000888888";
    struct T{
        char type[5];
        char acc[7];
        char passwd[7];	
    };
    //字符串地址解释为结构体地址
    T* pt = reinterpret_cast<T*>(text);
    cout << pt->type << endl;   //0001
    cout << pt->acc << endl;    //123456
    cout << pt->passwd << endl; //888888
    return 0;
}
