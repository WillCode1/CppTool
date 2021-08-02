//构造函数和对象实例化
#include <iostream>
using namespace std;
/*struct*/
class Student{
public:
    //构造函数
    Student(const string& name,int age,int no){
        cout << "Student::Student(..)" << endl;
        m_name = name;
        m_age = age;
        m_no = no;
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
    //对象实例化
    //Student s("张飞",28,10011);
    Student s = Student("张飞",28,10011);
    s.who();

    //栈区创建多个对象
    Student sa[3] = {Student("赵云",27,10012),
        Student("关羽",30,10013),
        Student("刘备",35,10010)};
    sa[0].who();
    sa[1].who();
    sa[2].who();
    
    //在堆区创建单个对象
    Student* ps = new Student("曹操",38,10086);
    ps->who();
    delete ps;
    ps = NULL;

    //在堆区创建多个对象
    ps = new Student[3]{
        Student("司马懿",40,10087),
        Student("张辽",35,10088),
        Student("曹仁",28,10089)};
    ps[0].who();
    ps[1].who();
    ps[2].who();
    delete[] ps;
    ps = NULL;
    return 0;
}
