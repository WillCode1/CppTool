//构造函数重载,和缺省构造函数
#include <iostream>
using namespace std;

class Student{
public:
    //1.缺省构造函数
    Student(const string& name/*="无名"*/,int age=0,int no=0){
        cout << "Student::Student(..)" << endl;
        m_name = name;
        m_age = age;
        m_no = no;
    }
    //2.另一个重载的构造函数
    Student(void){
        cout << "Student::Student(void)" << endl;
        m_name = "匿名";
        m_age = 0;
        m_no = 0;
    }
    void who(void){
        cout << "我叫" << m_name << ",今年" << 
            m_age << "岁，学号是"<< m_no << endl;
    }
private:
    string m_name;
    int m_age;
    int m_no;
};
int main(void)
{
    Student s;
    //Student s = Student();
    s.who();
    Student s1("张飞");
    s1.who();
    return 0;
}
