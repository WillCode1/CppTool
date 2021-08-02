//优先队列的使用
#include <iostream>
#include <queue>
#include <vector>
using namespace std;
//方法1
class IntCmp{
public:
    bool operator()(int a, int b)const{
        //容器内部，使用'<'比较，所以这里使用'>'
        return a > b;
    }
};
//方法2
//包装数据的类，然后重载'<'运算符
class Integer{
public:
    Integer(int arg):m_var(arg){}
    bool operator<(Integer const& i)const{
//	return m_var < i.m_var;
        return m_var > i.m_var;
    }
    friend ostream& operator<<(ostream& os, Integer const& i){
        return os << i.m_var;
    }
private:
    int m_var;
};
int main(void){
//  priority_queue<int> pq;
//  priority_queue<int, vector<int>, IntCmp> pq;
    priority_queue<Integer> pq;
    pq.push(40);
    pq.push(50);
    pq.push(30);
    pq.push(60);
    pq.push(20);
    pq.push(60);
    pq.push(40);
    while(!pq.empty()){
        cout << pq.top() << ' ' <<flush;
        pq.pop();
    }
    cout << endl;
    return 0;
}
