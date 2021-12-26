//单例模式：饿汉式
#include <iostream>
using namespace std;


class A{
public:
    static A& getInstance(void){
        return s_instance;
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
    static A s_instance;
};
A A::s_instance(1234);

int main(void)
{
    A& a1 = A::getInstance();
    A& a2 = A::getInstance();

    a1.print();
    a2.print();
    
	return 0;
}

