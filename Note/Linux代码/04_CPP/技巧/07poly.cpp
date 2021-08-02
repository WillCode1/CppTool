//简单工厂方法模式
#include <iostream>
using namespace std;

class PDFParser{
public:
    void prase(const char* pdffile){
        //解析出一个矩形
        onRect();
        //解析出一个圆形
        onCircle();
        //解析出一行文本
        onText();
        //解析出一张图片
        onImage();
    }
private:
    virtual void onRect(void) = 0;
    virtual void onCircle(void) = 0;
    virtual void onText(void) = 0;
    virtual void onImage(void) = 0;
};
class PDFRender:public PDFParser{
    void onRect(void){
        cout << "绘制一个矩形" << endl;
    }
    void onCircle(void){
        cout << "绘制一个圆形" << endl;
    }
    void onText(void){
        cout << "显示一行文本" << endl;
    }
    void onImage(void){
        cout << "显示一张图片" << endl;
    }
};
int main(void)
{
    PDFRender render;
    render.prase("something.pdf");
    return 0;
}

