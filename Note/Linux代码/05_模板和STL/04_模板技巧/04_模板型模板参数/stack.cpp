//模板型模板参数和模板型成员变量
#include <iostream>
using namespace std;
// 数组模板
template<typename T>
class Vector{
public:
    void push_back(T const& data){
        cout << "向数组尾端压入数据" << endl;
    }
    void pop_back(void){
        cout << "从数组尾端弹出数据" << endl;
    }
};
// 链表模板
template<typename T>
class List{
public:
    void push_back(T const& data){
        cout << "向链表尾端压入数据" << endl;
    }
    void pop_back(void){
        cout << "从链表尾端弹出数据" << endl;
    }
};
// 堆栈模板
template<typename T, template<typename> class C>
class Stack{
public:
    // 压入
    void push(T const& data){
        m_c.push_back(data);
    }
    // 弹出
    void pop(void){
        m_c.pop_back();
    }
private:
    C<T> m_c;
//  List<T> m_c;//模板型成员变量
//  C m_c;
};

int main(void){
//  Stack<int, Vector<int> > siv;
    Stack<int, Vector> siv;
    siv.push(100);
    siv.pop();
//  Stack<int, List<int> > sil;
    Stack<int, List> sil;
    sil.push(100);
    sil.pop();
    return 0;
}
