//Cµ÷C++
#include <iostream>
using namespace std;

//extern "C" int add(int x, int y);
extern "C"{
    void add(int x, int y);
    void sub(int x, int y);
}

void add(int x, int y){
    cout << x << "+" << y << "=" << x + y << endl;
}
void sub(int x, int y){
    cout << x << "-" << y << "=" << x - y << endl;
}
