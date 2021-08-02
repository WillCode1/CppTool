//向量容器的实例化(数组)
#include <iostream>
#include <vector>
using namespace std;
void print(vector<int> const& vi){
    cout << "字节数：" << sizeof(vi) << endl;
    size_t size = vi.size();
    cout << "元素数：" << size << endl;
    cout << "元  素：" << flush;
    for(size_t i = 0; i < size; ++i)
        cout << vi[i] << ' ';
    cout << endl;
}
int main(void){
    vector<int> v1;
    print(v1);
    vector<int> v2(5);
    print(v2);
    vector<int> v3(5, 10);
    print(v3);
    int ai[5] = {10, 20, 30, 40, 50};
    vector<int> v4(ai, ai + 5);
    print(v4);
//  vector<int> v5(ai + 1, ai + 4);
    vector<int> v5(&ai[1], &ai[4]);
    print(v5);
    return 0;
}
