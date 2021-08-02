//缺省构造函数
#include <iostream>
using namespace std;

class Teacher{
public:
    /*Teacher(void){
        cout << "缺省构造函数" << endl;
    }*/
    Teacher(const string& name="无名",int age=0){
        m_name = name;
        m_age = age;
    }
    void who(void){
        cout << m_name << ',' << m_age << endl;
    }
private:
    string m_name;
    int m_age;
};

int main(void)
{
    Teacher t;
    t.who();
    return 0;
}
