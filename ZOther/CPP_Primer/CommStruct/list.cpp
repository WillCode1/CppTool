//实现单链表中的各种操作

#include "pch.h"
#include <stdio.h>


class Node
{
public:
	Node(int data = 0) :data_(data), next_(nullptr) {}

public:
	int		data_;
	Node*	next_;
};

class List
{
public:
	List() :head_(nullptr), cnt_(0) {}

	void insert(int pos, int data);
	void remove(int pos);
	void travel() const;
	void clear() { while (head_ != nullptr) remove(0); }
	void push_head(int data) { insert(0, data); }
	void push_tail(int data) { insert(cnt_, data); }
	int size() const { return cnt_; }
	bool empty() const { return head_ == nullptr; }

private:
	Node*	head_;
	int		cnt_;
};

void List::insert(int pos, int data)
{
	if (pos < 0 || pos > cnt_)
		pos = cnt_; //插入到尾部

	const auto insert = new Node(data);

	if (0 == pos)
	{
		insert->next_ = head_;
		head_ = insert;
	}
	else
	{
		auto position = head_;
		for (auto i = 1; i < pos; i++)
			position = position->next_;

		insert->next_ = position->next_;
		position->next_ = insert;
	}
	cnt_++;
}

void List::remove(int pos)
{
	if (pos < 0 || pos >= cnt_)
	{
		printf("坐标不合法，删除节点失败\n");
		return;
	}
	if (0 == pos)
	{
		auto remove = head_;
		head_ = head_->next_;
		delete remove;
		remove = nullptr;
	}
	else
	{
		auto previous = head_;
		for (auto i = 1; i < pos; i++)
			previous = previous->next_;

		auto remove = previous->next_;
		previous->next_ = previous->next_->next_;
		delete remove;
		remove = nullptr;
	}
	cnt_--;
}

void List::travel() const
{
	Node* p = head_;
	printf("链表中的元素有：");
	while (p != nullptr)
	{
		printf("%d ", p->data_);
		p = p->next_;
	}
	printf("\n");
}


int main()
{
	//创建链表，并且进行初始化
	List list;

	//插入新节点，并且进行遍历
	list.push_head(11);
	list.travel();		//11
	list.push_head(22);
	list.travel();		//22 11
	list.push_head(33);
	list.travel();		//33 22 11
	printf("---------------\n");

	printf("链表中节点的个数是：%d\n", list.size());
	printf("%s\n", list.empty() ? "链表为空" : "链表不为空");
	printf("--------------------\n");

	list.travel();		//33 22 11
	list.insert(-2, 44);
	list.travel();		//33 22 11 44
	list.insert(0, 55);
	list.travel();		//55 33 22 11 44
	list.insert(2, 66);
	list.travel();		//55 33 66 22 11 44
	list.insert(6, 77);
	list.travel();		//77在最后
	printf("------------------\n");

	list.remove(-2);	//删除失败
	list.remove(0);
	list.travel();		//33 66 22 11 44 77
	list.remove(1);
	list.travel();		//33 22 11 44 77
	list.remove(5);		//删除失败
	list.travel();		//33 22 11 44 77
	list.remove(4);
	list.travel();		//33 22 11 44
	printf("-------------------\n");

	list.clear();
	list.travel();
	return 0;
}

