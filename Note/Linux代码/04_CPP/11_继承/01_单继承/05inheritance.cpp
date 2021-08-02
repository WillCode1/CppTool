//向上造型不适用于私有和保护继承
#include <iostream>
using namespace std;

class Base{};
class Derived:private Base{};
int main(void)
{
    Derived d;
    //Base* pb = &d;
    Base* pb = static_cast<Base*>(&d);
    return 0;
}
