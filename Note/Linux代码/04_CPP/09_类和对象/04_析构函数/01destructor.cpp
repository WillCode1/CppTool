//析构函数
#include <iostream>
using namespace std;

class Integer{
public:
    Integer(int data = 0):m_data(new int(data)){
        //m_data = new int(data);
    }
    //析构函数，在对象被销毁时自动调用
    ~Integer(void){
        cout << "析构函数" << endl;
        delete m_data;
    }
    int get(void)const{
        return *m_data;
    }
private:
    int* m_data;
};
int main(void)
{
    Integer i(100);
    cout << i.get() << endl;//100
    return 0;
}

