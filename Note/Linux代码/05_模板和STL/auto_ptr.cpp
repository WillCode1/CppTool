//智能指针的局部特化
#include <cstdio>
#include <iostream>
#include <memory>
using namespace std;
class A {
public:
    A(int data = 0):m_data(data){
        cout << "A构造：" << this <<endl;
    }
    ~A(void){
        cout << "A析构：" << this <<endl;
    }
    void foo(void)const{
        cout << m_data << endl;
    }
    int m_data;
};
template<typename T>
class AutoPtr{
public:
    AutoPtr(T* p = NULL):m_p(p){}
    AutoPtr(AutoPtr& that):m_p(that.release()){}
    AutoPtr& operator=(AutoPtr& that){
        if(&that != this)
            reset(that.release());
        return *this;
    }
    ~AutoPtr(void){
        delete m_p;
    }
    T& operator*(void)const{
        return *m_p;
    }
    T* operator->(void)const{
        return &**this;
    }
private:
    //实际的拷贝转移部分
    T* release(void){
        T* p = m_p;
        m_p = NULL;
        return p;
    }
    void reset(T* p){
        if(p != m_p){
            delete m_p;
            m_p = p;
        }
    }
    T* m_p;
};
//局部特化
template<typename T>
class AutoPtr<T[]>{
public:
    AutoPtr(T* p = NULL):m_p(p){}
    AutoPtr(AutoPtr& that):m_p(that.release()){}
    AutoPtr& operator=(AutoPtr& that){
        if(&that != this)
            reset(that.release());
        return *this;
    }
    ~AutoPtr(void){
        delete[] m_p;
    }
    T& operator*(void)const{
        return *m_p;
    }
    T* operator->(void)const{
        return &**this;
    }
private:
    T* release(void){
        T* p = m_p;
        m_p = NULL;
        return p;
    }
    void reset(T* p){
        if(p != m_p){
            delete m_p;
            m_p = p;
        }
    }
    T* m_p;
};
int foo(void){
//  auto_ptr<A> pa(new A);
//  smart_ptr<A> pa(new A); // C++11
    AutoPtr<A> pa(new A);
    pa->m_data = 1000;
    (*pa).foo(); // 1000
    AutoPtr<A> pb = pa; // 转移拷贝构造
    ++pb->m_data;
    (*pb).foo(); // 1001
    AutoPtr<A> pc(new A(2000));
    pb = pc;    // 转移拷贝赋值
    ++pb->m_data;
    (*pb).foo(); // 2001
    // ...
    FILE* fp = fopen("none", "r");
    if(! fp){
        perror("fopen");
//	delete pa;
        return -1;
    }
    // ...
    fclose(fp);
    // ...
//  delete pa;
    return 0;
}

int main(void){
    foo();
//  auto_ptr<A> pa(new A[3]);
    AutoPtr<A[]> pa(new A[3]);
    return 0;
}
