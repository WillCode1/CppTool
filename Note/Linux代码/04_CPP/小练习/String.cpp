//练习：写一个String类,要求可以实现深拷贝
#include <iostream>
#include <cstring>
using namespace std;

class String{
public:
    //构造函数
    String(const char* str = ""):
        m_str(strcpy(new char[strlen(str)+1],str)){}
    //析构函数
    ~String(void){
        delete[] m_str;
        m_str = NULL;
    }
    //拷贝构造:深拷贝
    String(const String& that):
        m_str(strcpy(new char[strlen(that.m_str)+1],that.m_str)){}
    //深拷贝赋值
    String& operator=(const String& that){
        if(&that != this){
            /*小鸟：
            delete[] m_str;
            m_str = new char[strlen(that.m_str)+1];
            strcpy(m_str,that.m_str);*/
            /*大鸟*/
            char* str = new char[strlen(that.m_str)+1];
            delete[] m_str;
            m_str = strcpy(str,that.m_str);
            /*老鸟
            //复用深拷贝拷贝构造和析构
            //交换地址后,临时变量遇到'}'被析构,有普通成员时效率不高
            String temp(that);
            swap(m_str,temp.m_str);*/
        }
        return *this;
    }

    //提供访问接口
    const char* c_str(void)const{
        return m_str;
    }
private:
    char* m_str;
};
int main(void)
{
    String s1("hello world");
    cout << s1.c_str() << endl;
    String s2(s1);
    cout << s2.c_str() << endl;
    String s3("hello C++");
    s2 = s3;//拷贝赋值
    cout << s2.c_str() << endl;//hello C++
    return 0;
}
