//成员变量指针
#include <iostream>
#include <cstdio>
using namespace std;
class Student{
public:
    Student(const string& name):m_name(name){}
    int m_a;
    double m_d;
    string m_name;
};
int main(void)
{
    //定义成员指针
    string Student::*pm = &Student::m_name;
    Student s1("张飞");
    cout << s1.*pm << endl;

    Student* ps = &s1;
    cout << ps->*pm << endl;

    //成员变量指针本质就是类中特定成员变量在对象中的相对地址
    printf("pme:%p\n",pm);
    printf("&s1:%p\n",&s1);
    printf("&s1.m_name:%p\n",&s1.m_name);
    return 0;
}



