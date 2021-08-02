//队列容器的使用
#include <iostream>
#include <queue>
#include <list>
using namespace std;
int main(void){
//  queue<string, list<string> > qs;
    queue<string> qs;
    qs.push ("我们");
    qs.push ("喜欢");
    qs.push ("C++!");
    while(!qs.empty()){
        cout << qs.front() << flush;
        qs.pop();
    }
    cout << endl;
    return 0;
}
