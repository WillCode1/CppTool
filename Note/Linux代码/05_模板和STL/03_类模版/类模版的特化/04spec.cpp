//类模版的特化
#include <cstring>
#include <iostream>
using namespace std;
// 通用版本
template<typename T>
class Comparator {
public:
    Comparator(T x, T y):m_x(x),m_y(y){}
    T max(void)const{
        return m_x < m_y ? m_y : m_x;
    }
#if 0
    char* max(void)const{
        return strcmp(m_x, m_y) < 0 ? m_y : m_x;
    }
#endif
private:
    T m_x, m_y;
};
// 针对char*类型的特化版本
#if 0
// 全类特化
template<>
class Comparator<char*> {
public:
    Comparator(char* x, char* y):m_x(x), m_y(y){}
    char* max(void)const{
        return strcmp(m_x, m_y) < 0 ? m_y : m_x;
    }
private:
    char* m_x, *m_y;
};
#endif
// 成员特化
template<>
char* Comparator<char*>::max(void)const{
    return strcmp(m_x, m_y) < 0 ? m_y : m_x;
}

int main(void){
    int a = 123, b = 345;
    char c[] = "hello", d[] = "world";

    Comparator<int> ci(a, b);
    cout << ci.max () << endl;
    Comparator<char*> cs(c, d);
    cout << cs.max () << endl;
    return 0;
}
