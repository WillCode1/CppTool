//函数模版演示
#include <iostream>
using namespace std;
// 函数模板/模板函数
// 将类型参数化，T被称为类型参数
template<typename T>
T max(T x, T y){
    return x < y ? y : x;
}

int main (void){
    int a = 123, b = 456;
//    cout << ::max<int>(a, b) << endl;
    cout << ::max(a, b) << endl;
    // 456
    double c = 1.3, d = 4.6;
//    cout << ::max<double>(c, d) << endl;
    cout << ::max(c, d) << endl;
    // 4.6
    string e = "world", f = "hello";
//    cout << ::max<string>(e, f) << endl;
    cout << ::max(e, f) << endl;
    // world
    char g[] = "world", h[] = "hello";
    //编译器会隐式推断为char*类型,这样就变成了数组地址比高低
    cout << ::max<string>(g, h) << endl;
    cout << ::max(g, h) << endl;
    return 0;
}
