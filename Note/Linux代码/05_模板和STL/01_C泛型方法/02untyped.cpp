//宏在某种程度上摆脱静态类型的约束
//但是有类型转换的风险
#include <iostream>
using namespace std;
#define max(x, y) ((x) < (y) ? (y) : (x))

int main (void) {
    int a = 123, b = 456;
    cout << max (a, b) << endl;
    // 456
    double c = 1.3, d = 4.6;
    cout << max (c, d) << endl;
    // 4.6
    string e = "hello", f = "world";
    cout << max (e, f) << endl;
    // world
    char g[] = "hello", h[] = "world";
    cout << max (g, h) << endl;
    // ?
    return 0;
}
