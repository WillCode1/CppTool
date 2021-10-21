#pragma once
/**
 * Definition for singly-linked list.
 */
struct ListNode {
	int val;
	ListNode *next;
	ListNode(int x = 0, ListNode *next = nullptr) : val(x), next(next) {}

	// 链表反序
	ListNode* reverseList(ListNode* head);
	ListNode* reverseList2(ListNode* head);

	// 删除链表的倒数第 N 个结点
	int cur = 0;
	ListNode* removeNthFromEnd(ListNode* head, int n);
	ListNode* removeNthFromEnd2(ListNode* head, int n);

private:
	ListNode* recursion(ListNode* pre, ListNode* cur);
};

/*
方法汇总：
	1.双指针法
	2.快慢指针(同步长，先起步；不同步长，同时起步)
	3.递归
*/
