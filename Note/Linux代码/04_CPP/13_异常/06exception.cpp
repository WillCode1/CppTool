//子类的异常捕获写到前面
#include <iostream>
using namespace std;

class A{};
class B:public A{};

void func(void){
    //throw B();
    throw A();
}
int main(void){
    try{
        func();
    }
    //子类的异常捕获写到前面
    catch(B& ex){
        cout << "捕获到异常B" << endl;
        return -1;
    }
    catch(A& ex){
        cout << "捕获到异常A" << endl;
        return -1;
    }
    return 0;
}

