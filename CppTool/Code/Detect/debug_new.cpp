#include<iostream>
#include <mutex>
#include <string.h>

//---------------------------------------------------------------
// 内存记录
//---------------------------------------------------------------
struct MemInfo
{
	void* ptr;
	const char* file;
	unsigned int line;
	unsigned int size;
	MemInfo* next;
};

//---------------------------------------------------------------
// 内存记录栈 
//---------------------------------------------------------------
struct MemStack
{
	MemStack() : head(NULL) { }

	~MemStack()
	{
		MemInfo* tmp;
		while (head != NULL)
		{
			free(head->ptr);
			tmp = head->next;
			free(head);
			head = tmp;
		}
	}

	void insertPtr(void* ptr, const char* file, unsigned int line, unsigned int size)
	{
		MemInfo* node = (MemInfo*)malloc(sizeof(MemInfo));
		node->ptr = ptr;
		node->file = file;
		node->line = line;
		node->size = size;
		node->next = head;
		head = node;
	}

	void deletePtr(void* ptr)
	{
		MemInfo* node = head;
		MemInfo* pre = NULL;

		while (node != NULL && node->ptr != ptr)
		{
			pre = node;
			node = node->next;
		}

		if (node == NULL) return;


		if (pre == NULL) // 删除的是head 
		{
			head = node->next;
		}
		else
		{
			pre->next = node->next;
		}

		free(node);
	}

	void printInfo()
	{
		if (head == NULL)
		{
			std::cout << "all memory is ok" << std::endl;
			return;
		}

		std::cout << "memory leak" << std::endl;
		MemInfo* node = head;
		while (node != NULL)
		{
			std::cout << "file: " << node->file << " , " << "line: " << node->line << " , "
				<< "addr: " << node->ptr << "size: " << node->size << std::endl;
			node = node->next;
		}
	}

private:
	MemInfo* head;
};


static MemStack mem_stack;
std::mutex g_mutex_debug_new;

void memInit()
{
	memset(&mem_stack, 0, sizeof(MemStack));
}

void memStackPrint()
{
	mem_stack.printInfo();
}

//---------------------------------------------------------------
// 重载new,new[],delete,delete[] 
//---------------------------------------------------------------
void* operator new(size_t size, const char* file, int line)
{
	void* ptr = malloc(size);
	std::lock_guard<std::mutex> lock(g_mutex_debug_new);
	mem_stack.insertPtr(ptr, file, line, size);
	return ptr;
}

void* operator new[](size_t size, const char* file, int line)
{
	return operator new(size, file, line);
}

void operator delete(void* ptr)
{
	free(ptr);
	std::lock_guard<std::mutex> lock(g_mutex_debug_new);
	mem_stack.deletePtr(ptr);
}

void operator delete[](void* ptr)
{
	operator delete(ptr);
	mem_stack.deletePtr(ptr);
}

//---------------------------------------------------------------
// 使用宏将带测试代码中的new和delte替换为重载的new和delete 
//---------------------------------------------------------------
//#define new new(__FILE__,__LINE__)


