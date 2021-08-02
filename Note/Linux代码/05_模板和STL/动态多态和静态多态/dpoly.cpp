//动态多态,基于虚函数和动态绑定的多态
#include <iostream>
using namespace std;
class Shape{
public:
    virtual void draw(void)const = 0;
};
class Rect:public Shape{
public:
    void draw(void)const{
        cout << "绘制矩形" << endl;
    }
};
class Circle:public Shape{
public:
    void draw(void)const{
        cout << "绘制圆形" << endl;
    }
};
// 多态函数
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
