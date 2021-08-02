//析构函数中最好不要抛出异常
#include <iostream>
using namespace std;
class A{
public:
    void func(void){
        throw -1;
    }
    ~A(void){
	//不可避免,则在析构函数内部捕获处理
        try{
            throw -2;
        }
        catch(int& ex){
            cout << "捕获到异常：" << ex << endl;
        }
    }
};
int main(void)
{
    try{
        A a;
        a.func();
    }
    catch (int& ex){
        cout << "捕获到异常：" << ex << endl;
        return -1;
    }	
    return 0;
}
