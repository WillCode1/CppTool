//引用返回局部变量是危险的
#include <iostream>
using namespace std;

struct A{
    int data;
    //使用常引用,防止数据被外部修改
    const int& foo(void){
        return data;
    }
    int& bar(void){
        int data1 = 200;
        cout << "&data1:" << &data1 << endl;
        //返回局部变量引用,可能会出错
        return data1;
    }
    int& hum(void){
        int data2 = 300;
        cout << "&data2:" << &data2 << endl;
        return data2;
    }
};

int main(void)
{
    A a = {0};
    //a.foo() = 100;    //error
    
    int& data = a.bar();
    cout << data << endl;//200
    
    a.hum();
    cout << data << endl;//300
    return 0;
}
