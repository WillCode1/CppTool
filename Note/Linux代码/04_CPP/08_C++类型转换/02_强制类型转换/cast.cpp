//C++支持的两种强制类型转换
#include <iostream>
using namespace std;

int main(){
    int a = 70, b = 100;
    
    //1.C风格的强制转换
    char ch = (char)a;
    cout << ch << endl;

    //2.C++风格的强制转换
    ch = char(b);
    cout << ch << endl;
    return 0;
}
