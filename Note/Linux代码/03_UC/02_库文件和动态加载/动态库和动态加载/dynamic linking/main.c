#include <stdio.h>
#include "process.h"
#include "tmath.h"
int main(void){
	int a=8,b=2;
	math_t m[4]={add,sub,div,mul};

	printf("a+b=%d\n",process(add,a,b));
	printf("a-b=%d\n",process(m[1],a,b));
	printf("a*b=%d\n",mul(a,b));
	printf("a/b=%d\n",div(a,b));

	return 0;
}
