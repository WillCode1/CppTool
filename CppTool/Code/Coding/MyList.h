//实现单链表中的各种操作
#pragma once
#include <stdio.h>

class Node
{
public:
    Node(int data = 0) :data_(data), next_(nullptr) {}

public:
    int		data_;
    Node*	next_;
};

class MyList
{
public:
    MyList() :head_(nullptr), cnt_(0) {}

    void insert(int pos, int data);
    void remove(int pos);
    void travel() const;
    void reverse();
    void clear() { while (head_ != nullptr) remove(0); }
    void push_head(int data) { insert(0, data); }
    void push_tail(int data) { insert(cnt_, data); }
    int size() const { return cnt_; }
    bool empty() const { return head_ == nullptr; }

private:
    Node*	head_;
    int		cnt_;
};

void testMyList();
