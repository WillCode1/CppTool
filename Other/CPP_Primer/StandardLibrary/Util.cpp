#include "pch.h"
#include "Util.h"

ostream& operator<<(ostream& os, const A& a) {
	return os << a._a;
}