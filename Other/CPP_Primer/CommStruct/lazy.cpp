//单例模式：懒汉式
#include <iostream>
using namespace std;

class A{
public:
    static A& getInstance(void){
        if(!s_instance)
			s_instance = new A(1234);
        ++s_counter;
        return *s_instance;
    }

    void release(void){
        if(s_counter && --s_counter == 0){
            delete this;
            s_instance = NULL;
        }
    }
    void print(void){
        cout << m_data << endl;
    }

private:
	A(int data = 0) :m_data(data) {}
	~A(void) = default;
	A(const A&) = delete;

private:
    int m_data;
    static A* s_instance;
    static int s_counter;
};
A* A::s_instance = NULL;
int A::s_counter = 0;

int main(void)
{
    A& a1 = A::getInstance();//-->new A
    A& a2 = A::getInstance();

    a1.print();
    a1.release();
    a2.print();
    a2.release();   //--delete

    return 0;
}

