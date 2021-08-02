//单例模式：懒汉式
#include <iostream>
using namespace std;
class A{
public:
    //3)提供该对象的访问接口
    static A& getInstance(void){
        //4)有人访问才开辟空间,创建一个唯一对象
        if(!s_instance)
			s_instance = new A(1234);
        ++s_counter;
        return *s_instance;
    }
    void release(void){
        //使用计数-1,为0时,才销毁唯一对象
        if(s_counter && --s_counter == 0){
            delete this;    //自销毁
            s_instance = NULL;
        }
    }
    void print(void){
        cout << m_data << endl;
    }
private:
    //1)私有化所有的构造函数
    A(int data = 0):m_data(data){
        cout << "构造函数" << endl;
    }
    ~A(void){
        cout << "析构函数" << endl;
    }
    A(const A&);
    int m_data;
    //2)懒汉模式,进程启动时只准备一个指针
    static A* s_instance;
    //记录使用单例对象的人数
    static int s_counter;
};
A* A::s_instance = NULL;
int A::s_counter = 0;

int main(void)
{
    A& a1 = A::getInstance();//-->new A
    A& a2 = A::getInstance();
    cout << &a1 << ' ' << &a2 << endl;
    a1.print();
    a1.release();
    a2.print();
    a2.release();   //--delete
    return 0;
}

