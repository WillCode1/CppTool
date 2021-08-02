//用宏替换函数实现的代码
#include <iostream>
using namespace std;
#define MAX(T)  T max_##T(T x, T y){return x < y ? y : x;}
#define max(T) max_##T  //把max和T粘起来
MAX (int)
/*
int max_int (int x, int y) {
    return x < y ? y : x;
}
*/
MAX (double)
MAX (string)

int main (void) {
    int a = 123, b = 456;
    cout << max(int) (a, b) << endl;
//  cout << max_int (a, b) << endl;
    // 456
    double c = 1.3, d = 4.6;
    cout << max(double) (c, d) << endl;
//  cout << max_double (c, d) << endl;
    // 4.6
    string e = "hello", f = "world";
    cout << max(string) (e, f) << endl;
//  cout << max_string (e, f) << endl;
    // world
    char g[] = "hello", h[] = "world";
    cout << max(string) (g, h) << endl;
//  cout << max_string (g, h) << endl;
    // world
    return 0;
}
