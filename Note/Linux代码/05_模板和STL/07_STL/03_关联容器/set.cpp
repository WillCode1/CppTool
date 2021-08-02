//集合和多重集合的使用
#include <iostream>
#include <set>
using namespace std;

int main(void){
    set<int> si;
    si.insert(10);
    si.insert(20);
    si.insert(30);
    si.insert(10);
    si.insert(10);
    si.insert(20);
    si.insert(20);
    si.insert(10);
    si.insert(30);
    si.insert(30);
    typedef set<int>::iterator IT;
    for(IT it = si.begin(); it != si.end(); ++it)
        cout << /*++*/*it << ' ';
    cout << endl;
    multiset<int> msi;
    msi.insert(10);
    msi.insert(20);
    msi.insert(30);
    msi.insert(10);
    msi.insert(10);
    msi.insert(20);
    msi.insert(20);
    msi.insert(10);
    msi.insert(30);
    msi.insert(30);
    typedef multiset<int>::iterator MIT;
    for(MIT it = msi.begin(); it != msi.end(); ++it)
        cout << /*++*/*it << ' ';
    cout << endl;
    return 0;
}
