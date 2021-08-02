//向量容器的使用
#include <iostream>
#include <vector>
using namespace std;
void print(vector<int> const& vi){
    //正向只读迭代器const_iterator
    for(vector<int>::const_iterator it = vi.begin(); it != vi.end(); ++it)
        cout << /*++*/*it << ' ';
    cout << endl;
}
void rprint(vector<int> const& vi){
    //反向只读迭代器const_reverse_iterator
    for(vector<int>::const_reverse_iterator it = vi.rbegin(); it != vi.rend(); ++it)
        cout << *it << ' ';
    cout << endl;
}
int main(void){
    vector<int> vi;
    for(int i = 1; i <= 10; ++i)
        vi.push_back(i);
    size_t size = vi.size();
    for(size_t i = 0; i < size; ++i)
        cout << vi[i] << ' ';
    cout << endl;

    //迭代器的使用
    //正向可写迭代器iterator
    for(vector<int>::iterator it = vi.begin(); it != vi.end(); ++it)
        cout << *it << ' ';
    cout << endl;
    print(vi);

    //反向可写迭代器reverse_iterator
    for(vector<int>::reverse_iterator it = vi.rbegin(); it != vi.rend(); ++it)
        cout << *it << ' ';
    cout << endl;
    rprint(vi);
   
    cout << "------------------------" << endl;
    //随机访问(下标)
#if 0
    vi[4] = 50;
#else
    vector<int>::iterator it1 = vi.begin();
    //++++++++it1 = 50;
    *(it1 + 4) = 50;
#endif
    print(vi);

    // 随机迭代器的大小比较
    vector<int>::iterator it2 = vi.end() - 1;
    // 越靠近起始位置的迭代器越小, 越靠近终止位置的迭代器越大
    cout << boolalpha << (it1 < it2) << endl; // true
    cout << it2 - it1 << endl; // 9
    
    vector<int>::reverse_iterator it3 = vi.rbegin(),
        it4 = vi.rend() - 1;
    cout << (it4 < it3) << endl; // false
    cout << it3 - it4 << endl; // -9
    
    vector<int> vn;
    vn.push_back(100);
    it1 = vn.begin();
    cout << *it1 << endl; // 100
    vn.push_back(200);
    //结构改变后，重置迭代器
    it1 = vn.begin();
    cout << *it1 << endl; // 100
    vn.clear();
    vn.push_back(11);
    vn.push_back(22);
    vn.push_back(44);
    vn.push_back(33);
    vn.push_back(55);
    print(vn);
    it1 = vn.begin();
    while(it1 != vn.end())
        if(*it1%2)
            ++it1;
        else
//	    vn.erase(it1);  //结构已改变
            it1 = vn.erase(it1);
    print(vn);
    return 0;
}
