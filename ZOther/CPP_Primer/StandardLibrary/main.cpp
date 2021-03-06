// C++ Standard Library：泛型算法、容器库、lambda表达式
// 容器库包括：顺序容器、容器适配器、迭代器、关联容器
//
#include "pch.h"
#include "SequenceContainer.h"
#include "Adaptor.h"
#include "GenericAlgorithm.h"
#include "AssociativeContainer.h"


int main()
{
#if 0
	Test_Container();

	// 顺序容器（SequenceContainer）
	Test_string();	
	Test_vector();
	Test_forward_list();
	Test_array();
#endif // Sequence

#if 0
	// 容器适配器（ContainerAdaptor）
	Test_Adaptor();
#endif // Adaptor

#if 0
	// GenericAlgorithm(泛型算法)
	Test_ReadOnly();
	Test_Write();
	Test_Lambda();
	Test_List();
#endif // GenericAlgorithm

#if 1
	// AssociativeContainer（关联容器）
//	Test();
	Test_Map();
	Test_Set();
#endif // Associative

	return 0;
}
