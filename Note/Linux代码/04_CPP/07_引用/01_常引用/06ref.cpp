//常引用,只读引用,万能引用
//既可以引用左值，右可以引用右值
#include <iostream>
using namespace std;
int foo(void){
    int a = 100;
    return a;
}
int main(void)
{
    //1.引用常量
    //int& r = 100;	//error
    const int& r = 100;	//ok
    cout << r << endl;	//100
    
    //2.引用临时变量
    int a = 10, b = 20;
    const int& c = a+b;
    cout << c << endl;

    //int& res = foo();	//error
    const int& res = foo();
    cout << "&res:" << &res << endl;

    //3.引用别的数据类型(隐式转换)
    char ch = 'A';
    //int& rch = ch;    //error
    const int& rch = ch;
    return 0;
}
