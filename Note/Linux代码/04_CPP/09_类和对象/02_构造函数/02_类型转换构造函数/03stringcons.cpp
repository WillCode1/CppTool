//字符指针通过类型转换构造函数转换成类类型
//多次隐式转换可能会报错
#include <iostream>
#include <cstring>
using namespace std;

class A{
public:
    A(void){}
    A(const char* str){
        //strcpy(buf, str);
        s = str;    //隐式转换
    }
    void print(void){
        //cout << buf << endl;
        cout << s << endl;
    }
private:
    //char buf[128];
    string s;
};

int main(){
    A a = "hello";  //隐式转换
    a.print();
    return 0;
}
