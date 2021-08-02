//前++/--,后++/--
#include <iostream>
using namespace std;

class Integer{
public:
	Integer(int i = 0):m_i(i){}
	void print(void)const{
		cout << m_i << endl;
	}
	//前++
	Integer& operator++(void){
		++m_i;
		return *this;
	}
	//前--
	friend Integer& operator--(Integer& i){
		--i.m_i;
		return i;
	}
	//后++
	const Integer operator++(int){
		Integer old = *this;
		//复用前++
		++(*this);//++m_i;
		return old;
	}
	//后--
	friend const Integer operator--(Integer& i,int){
		Integer old = i;
		--i;//--i.m_i;
		return old;
	}

private:
	int m_i;
};
int main(void)
{
	Integer i(100);
	Integer j = ++i;
	i.print();//101
	j.print();//101
	++++i;
	i.print();//103
	------i;
	i.print();//100
	cout << "=============" << endl;
	j = i++;
	i.print();//101
	j.print();//100

	j = i--;
	i.print();//100
	j.print();//101
	return 0;
}

