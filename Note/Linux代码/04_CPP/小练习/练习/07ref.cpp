//利用引用实现字符串的交换
#include <iostream>
using namespace std;

//'&'都放在类型后面
void swap1(const char*& x, const char*& y){
    const char *z = x;
    x = y;
    y = z;
}

int main(){
    const char* s1 = "hello";
    const char* s2 = "world";
    swap1(s1, s2);
    cout << s1 << endl;
    cout << s2 << endl;
}
