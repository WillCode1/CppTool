//向量容器的排序算法
#include <iostream>
#include <vector>
#include <algorithm> // 泛型算法
using namespace std;
template<typename T>
ostream& operator<<(ostream& os, vector<T> const& vec){
    for(typename vector<T>::const_iterator it = vec.begin(); it != vec.end(); ++it)
        os << *it << ' ';
    return os;
}
//sort()既可以通过传函数指针，给出排序规则
bool intCmp(int a, int b){
    return a > b;
}
//sort()也可以给临时对象调用重载的函数操作符()
class IntCmp{
public:
    bool operator()(int a, int b)const{
        return a > b;
    }
};
int main(void){
    int ai[] = {13, 27, 17, 19, 51, 33};
    vector<int> v1(ai, ai + sizeof(ai) / sizeof(ai[0]));
    cout << v1 << endl;
    sort(v1.begin(), v1.end());
    cout << v1 << endl;
    //sort(v1.rbegin(), v1.rend());
    sort(v1.begin(), v1.end(), intCmp);
    //sort(v1.begin(), v1.end(), IntCmp());
    cout << v1 << endl;
    return 0;
}
