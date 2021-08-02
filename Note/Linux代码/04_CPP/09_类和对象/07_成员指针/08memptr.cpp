//成员函数指针
#include <iostream>
using namespace std;

class Student{
public:
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
    void (Student::*pwho)(void) = &Student::who;
    Student s1("赵云",25);
    (s1.*pwho)();
    Student* ps1 = &s1;
    (ps1->*pwho)();
    return 0;
}


