//自定义的深拷贝
#include <iostream>
using namespace std;

class Integer{
public:
    Integer(int data = 0):m_data(new int(data)){
    }
    //自定义的拷贝构造函数
    //Integer(const Integer& that):m_data(new int(*that.m_data)){}
    Integer(const Integer& that){
        m_data = new int;
        *m_data = *that.m_data;
    }
    //自定义的拷贝赋值操作符函数
    Integer& operator=(Integer const& that){
        //1)防止自赋值
        if(this != &that){
			//2)释放旧资源
            delete m_data;
            //3)分配新资源拷贝数据
            m_data = new int(*that.m_data);
            //高手写法
            //Integer temp(that);
            //swap(m_data,temp.m_data);
        }
        //4)返回自引用
        return *this;
    }
    ~Integer(void){
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
    Integer i2(i);  //拷贝构造
    cout << i2.get() << endl;//100
    Integer i3;
    i3 = i2;        //拷贝赋值
    cout << i2.get() << endl;//100
    return 0;
}

