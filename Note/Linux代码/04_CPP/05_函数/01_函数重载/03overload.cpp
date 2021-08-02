//指针和引用的常属性可以影响重载
#include <iostream>
using namespace std;
#if 0
//基本类型的常属性无法影响重载
int max(int x, int y){
    cout << 5 << endl;
    return x;
}
int max(int const x, int const y){
    cout << 6 << endl;
    return x;
}
#else
int& max(int& x, int& y){
    cout << 1 << endl;
    return x;
}
int const& max(int const& x, int const& y){
    cout << 2 << endl;
    return x;
}
#endif
int* max(int* x, int* y){
    cout << 3 << endl;
    return x;
}
int const* max(int const* x, int const* y){
    cout << 4 << endl;
    return x;
}

int main(void){
    int p1 = 1, p2 = 2;
    const int s1 = p1, s2 = p2;
    max(p1, p2);
    max(s1, s2);

    max(&p1, &p2);
    max(&s1, &s2);
    return 0;
}
