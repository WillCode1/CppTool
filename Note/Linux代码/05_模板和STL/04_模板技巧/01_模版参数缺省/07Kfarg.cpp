//容器的递归实例化
#include <iostream>
using namespace std;
template<typename T = int, size_t S = 3>//非类型参数
class Array{
public:
    T& operator[](size_t i){
        return m_a[i];
    }
    T const& operator[](size_t i)const{
        return const_cast<Array&>(*this)[i];
    }
    size_t size(void)const{
        return sizeof(m_a) / sizeof(m_a[0]);
    }
private:
    T m_a[S];
};
template<typename T>
ostream& operator<<(ostream& os, Array<T> const& arr){
    size_t size = arr.size();
    for(size_t i = 0; i < size; ++i)
        os << '(' << arr[i]/*++*/ << ')';
    return os;
}

int main(void){
    int const /*volatile*/ col = 4;
    Array<Array<int, col>, (1+2)> g;
    for(int i = 0; i < g.size(); ++i)
        for(int j = 0; j < g[i].size(); ++j)
            g[i][j] = (i+1)*10+j+1;
    for(int i = 0; i < g.size(); ++i){
        for(int j = 0; j < g[i].size(); ++j)
            cout << g[i][j] << ' ';
        cout << endl;
    }
    return 0;
}
