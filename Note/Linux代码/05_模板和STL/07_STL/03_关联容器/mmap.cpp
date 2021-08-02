//多重映射的使用
#include <iostream>
#include <map>
using namespace std;

int main(void){
    multimap<string, int> msi;
    msi.insert(make_pair("张飞", 80));
    msi.insert(make_pair("赵云", 70));
    msi.insert(make_pair("关羽", 60));
    msi.insert(make_pair("张飞", 50));
    msi.insert(make_pair("赵云", 40));
    msi.insert(make_pair("关羽", 30));
    typedef multimap<string, int>::iterator IT;
    for(IT it = msi.begin(); it != msi.end(); ++it)
        cout << it->first << "：" << it->second << endl;
    cout << "--------------------------" << endl;
    //返回符合映射内容的迭代器范围,保存在pair对象中
    pair<IT, IT> res = msi.equal_range("张飞");
    int sum = 0;
    for(IT it = res.first; it != res.second; ++it)
        sum += it->second;
    cout << "张飞：" << sum << endl;
    return 0;
}
