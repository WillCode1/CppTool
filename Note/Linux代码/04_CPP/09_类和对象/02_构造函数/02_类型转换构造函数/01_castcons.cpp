//类型转换构造函数
#include <iostream>
using namespace std;

class Ingeter{
public:
    Ingeter(void){
        cout << "Ingeter::Ingeter()" << endl;
        m_data = 0;
    }
    //类型转换构造函数
    //explicit关键字：告诉编译器必须做显示转换
    explicit Ingeter(const int& i){
        m_data = i;
    }
    void print(void){
        cout << m_data << endl;
    }
private:
    int m_data;
};

int main(void){
    Ingeter i;
    i.print();  //0
    //隐式类型转换
    i = 100;
    i.print();  //100

    //显示类型转换
    //i = (Ingeter)200;
    i = Ingeter(200);
    //i = static_cast<Ingeter>(200);
    i.print();  //200
    return 0;
}
