// ÖÇÄÜÖ¸Õë

#include <iostream>
using namespace std;


template<class T>
class SharedPtr
{
public:
	SharedPtr(T *p) :ptr(p)
	{
		use_count = new int(1);

		cout << "Constructor is called!" << endl;
	}

	~SharedPtr()
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

	// Ç³¿½±´
	SharedPtr(const SharedPtr<T> &orig)
	{
		ptr = orig.ptr;
		use_count = orig.use_count;
		++(*use_count);

		cout << "Copy constructor is called!" << endl;
	}

	SharedPtr<T>& operator=(const SharedPtr<T> &rhs)
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

