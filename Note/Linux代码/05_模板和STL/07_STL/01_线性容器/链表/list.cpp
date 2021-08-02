//链表容器的使用
#include <iostream>
#include <list>
//#include <algorithm>
using namespace std;
void print(list<int> const& li){
    for(list<int>::const_iterator it = li.begin(); it != li.end(); ++it)
        cout << *it << ' ';
    cout << endl;
}
class IntCmp{
public:
    IntCmp(bool raise = true):m_raise(raise){}
    bool operator()(int a, int b)const{
        if(m_raise)
            return a < b;
        else
            return a > b;
    }
private:
    bool m_raise;
};
int main(void){
    int ai[] = {10,10,20,20,10,20,30,20,20,10,10};
    list<int> l1(ai, ai + sizeof(ai) / sizeof(ai[0]));
    print(l1);
//  sort(l1.begin(), l1.end());
    l1.sort(IntCmp());
    print(l1);
    l1.sort(IntCmp(false));
    print(l1);
    l1.unique();
    cout << "l1: ";
    print(l1);
    cout << "------------------" << endl;
    //链表剪切
    list<int> l2;
    l2.push_back(1000);
    l2.push_back(2000);
    l2.push_back(3000);
    l2.push_back(4000);
    l2.push_back(5000);
    cout << "l2: ";
    print(l2);
    list<int>::iterator pos = l1.begin();
    /*
    l1.splice(++pos, l2);
    *//*
    list<int>::iterator del = l2.begin();
    l1.splice(++pos, l2, ++++del);
    */
    list<int>::iterator begin = l2.begin(), end = l2.end();
    l1.splice(++pos, l2, ++begin, --end);
    cout << "l1: ";
    print(l1);
    cout << "l2: ";
    print(l2);
    cout << "------------------" << endl;
    //链表有序合并
    l1.clear();
    l1.push_back(100);
    l1.push_back(500);
    l1.push_back(700);
    l2.clear();
    l2.push_back(200);
    l2.push_back(300);
    l2.push_back(600);
    l2.push_back(800);
    l2.push_back(900);
    cout << "l1: ";
    print(l1);
    cout << "l2: ";
    print(l2);
    l1.merge(l2);
    cout << "l1: ";
    print(l1);
    cout << "l2: ";
    print(l2);
    return 0;
}
