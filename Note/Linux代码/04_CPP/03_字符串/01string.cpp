//字符串对象的实例化
#include <iostream>
#include <cstring>
using namespace std;

int main(void)
{
    //1.实例化一个string类型的对象
    //在栈中显式实例化
    string s1 = string("hello");
    cout << "s1=" << s1 << endl;    //hello
    //在栈中隐式实例化
    string s2("hello");
    cout << "s2=" << s2 << endl;    //hello
    //在堆中实例化
    string *s = new string("hello");
    cout << "*s=" << *s << endl;    //hello
    
    //2.C ==> C++(隐式转换过程):
    string s3 = "hello";
    cout << "s3=" << s3 << endl;    //hello
    
    //3.C++ ==> C(调用成员函数c_str()):
    //string --> const char*
    char const* pc = s3.c_str();
    cout << "pc=" << pc << endl;    //hello
    return 0;
}
