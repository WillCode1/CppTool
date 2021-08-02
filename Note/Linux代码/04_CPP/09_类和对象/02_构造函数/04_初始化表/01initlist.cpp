//初始化表
#include <iostream>
using namespace std;

class Student{
public:
    /*先定义成员变量，再对其赋初值
    Student(const string& name,int age){
        m_name = name;
        m_age = age;
    }*/
    //定义成员变量同时对其初始化
    Student(const string& name,int age):
        m_name(name),m_age(age){}
    void who(void){
        cout << m_name << ',' << m_age << endl;
    }
private:
    string m_name;
    int m_age;
};
int main(void)
{
    Student s("张飞",29);
    s.who();
    return 0;
}


