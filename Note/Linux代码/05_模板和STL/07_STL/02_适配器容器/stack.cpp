//堆栈容器的使用
#include <iostream>
#include <stack>
#include <vector>
#include <list>
using namespace std;
int main(void){
//  stack<string, vector<string> > ss;
//  stack<string, list<string> > ss;
    stack<string> ss;
    ss.push("C++!");
    ss.push("喜欢");
    ss.push("我们");
    while(!ss.empty()){
        cout << ss.top() << flush;
        ss.pop();
    }
    cout << endl;
    return 0;
}
