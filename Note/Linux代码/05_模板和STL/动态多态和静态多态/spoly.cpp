//静态多态,基于类模板和类型参数的多态
#include <iostream>
using namespace std;
class Rect{
public:
    void draw(void)const{
        cout << "绘制矩形" << endl;
    }
};
class Circle{
public:
    void draw(void)const{
        cout << "绘制圆形" << endl;
    }
};
// 多态函数
template<typename Shape>
void drawAny(Shape const& shape){
    shape.draw();
}
int main(void){
    Rect r;
    Circle c;
    drawAny(r);
    drawAny(c);
    return 0;
}
