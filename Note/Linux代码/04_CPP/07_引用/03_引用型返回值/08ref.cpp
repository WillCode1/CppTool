//可行的引用类型返回值
#include <iostream>
using namespace std;

int& foo(int &n){return n;}
int* foo(int *n){return n;}

int* bar(int &n){return &n;}
int& bar(int *n){return *n;}

//常左值引用返回带有右值属性的返回值
int const& hum(int* n){return *n;}
//以下写法勉强可以(编译警告),但没有必要
int* const& hum(int& n){return &n;}

int main(){
    int a = 10;
    foo(a) = 100;
    cout << a << endl;
    bar(&a) = 200;
    cout << a << endl;
    //hum(&a) = 300;
    return 0;
}
