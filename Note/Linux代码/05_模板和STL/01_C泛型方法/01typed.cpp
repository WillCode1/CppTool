//静态类型语言的缺点
//解决问题的方法受限与数据类型,不够通用
#include <iostream>
using namespace std;

int max_int(int x, int y){
    return x < y ? y : x;
}
double max_double(double x, double y){
    return x < y ? y : x;
}
string max_string(string x, string y){
    return x < y ? y : x;
}
int main(void){
    int a = 123, b = 456;
    cout << max_int (a, b) << endl;     //456

    double c = 1.3, d = 4.6;
    cout << max_double (c, d) << endl;  //4.6
    
    string e = "hello", f = "world";
    cout << max_string (e, f) << endl;  //world
    
    char g[] = "hello", h[] = "world";
    cout << max_string (g, h) << endl;  //world
    return 0;
}
