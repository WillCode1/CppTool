//重载解析,匹配的优先级
#include <iostream>
using namespace std;
//char->int,升级转换
void bar(int i){
    cout << "bar(1)" << endl;
}
//char->const char，常量转换
void bar(const char c){
    cout << "bar(2)" << endl;
}
//short-->char,降级转换
void func(char c){
    cout << "func(1)" << endl;
}
#if 1
//short-->int,升级转换
void func(int i){
    cout << "func(2)" << endl;
}
#endif
//short-->long long,过分升级转换,和降级转换同样差
void func(long long ll){
    cout << "func(3)" << endl;
}
//省略号匹配最差的
void hum(int i,...){
    cout << "hum(1)" << endl;
}
//double-->int,降级转换
void hum(int i,int j){
    cout << "hum(2)" << endl;
}

int main(void)
{
    char c;
    bar(c);
    short s = 10;
    func(s);
    hum(10,1.23);
    return 0;
}
