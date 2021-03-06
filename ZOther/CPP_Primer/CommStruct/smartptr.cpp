// 智能指针

#include <iostream>
using namespace std;


template<class T>
class SmartPtr
{
public:
	SmartPtr(T *p) :ptr(p)
	{
		use_count = new int(1);

		cout << "Constructor is called!" << endl;
	}

	~SmartPtr()
	{
		if (--(*use_count) == 0)
		{
			delete ptr;
			delete use_count;
			ptr = nullptr;
			use_count = nullptr;
		}

		cout << "Destructor is called!" << endl;
	}

	// 浅拷贝
	SmartPtr(const SmartPtr<T> &orig)
	{
		ptr = orig.ptr;
		use_count = orig.use_count;
		++(*use_count);

		cout << "Copy constructor is called!" << endl;
	}

	SmartPtr<T>& operator=(const SmartPtr<T> &rhs)
	{
		++(*rhs.use_count);

		if (--(*use_count) == 0)
		{
			delete ptr;
			delete use_count;

			cout << "Left side object is deleted!" << endl;
		}

		ptr = rhs.ptr;
		use_count = rhs.use_count;

		cout << "Assignment operator overloaded is called!" << endl;
		return *this;
	}

	T& operator*(void)const { return *ptr; }
	T* operator->(void)const { return &**this; }

private:
	T *ptr;
	int *use_count;
};


int main()
{
	// Test Constructor and Assignment Operator Overloaded
	SmartPtr<int> p1(new int(0));
	p1 = p1;
	// Test Copy Constructor
	SmartPtr<int> p2(p1);
	// Test Assignment Operator Overloaded
	SmartPtr<int> p3(new int(1));
	p3 = p1;

	cout << *p3 << endl;

	return 0;
}