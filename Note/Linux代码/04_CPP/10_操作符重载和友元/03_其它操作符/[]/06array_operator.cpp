//'[]'下标操作符重载
#include <iostream>
using namespace std;

class Array{
public:
    Array(size_t size):
        m_data(new int[size]),m_size(size){}
    ~Array(void){
        delete[] m_data;
        m_data = NULL;
    }
    //非常容器对象:[]
    int& operator[](size_t i){
        return m_data[i];
    }
    //常容器对象：[]
    int const& operator[](size_t i)const{
        //return m_data[i];
        //直接复用上面非常版本
        return const_cast<Array&>(*this)[i];
    }
private:
    int* m_data;
    size_t m_size;
};
int main(void)
{
    Array a(5);
    //非常容器返回左值
    a[0] = 10;              //a.operator[](0)
    cout << a[0] << endl;   //10

    //常容器返回右值
    const Array& ra = a;
    //ra[0] = 20;           //error
    cout << ra[0] << endl;  //10
    return 0;
}


