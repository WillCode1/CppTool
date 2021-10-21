#include "List.h"

// 链表反序：双指针法
ListNode * ListNode::reverseList(ListNode * head)
{
	// 迭代，时间复杂度O(n)。
	ListNode* one = nullptr;
	ListNode* two = head;

	while (two) {
		auto temp = two->next;
		two->next = one;
		one = two;

		two = temp;
	}

	return one;
}

// 链表反序: 递归
ListNode * ListNode::recursion(ListNode * pre, ListNode * cur)
{
	if (!cur->next)
	{
		cur->next = pre;
		return cur;
	}

	auto tail = recursion(cur, cur->next);
	cur->next = pre;
	return tail;
}

ListNode * ListNode::reverseList2(ListNode * head)
{
	if (!head)
	{
		return head;
	}
	return recursion(nullptr, head);
}

// 删除链表的倒数第 N 个结点: 递归
ListNode * ListNode::removeNthFromEnd(ListNode * head, int n) {
	if (!head)
		return nullptr;
	head->next = removeNthFromEnd(head->next, n);
	cur++;
	if (n == cur)
		return head->next;
	return head;
}

// 删除链表的倒数第 N 个结点: 快慢指针
ListNode * ListNode::removeNthFromEnd2(ListNode * head, int n) {
	ListNode *fast = head, *slow = head;
	while (n--)
	{
		fast = fast->next;
	}

	if (!fast)
	{
		return head->next;
	}

	while (fast && fast->next)
	{
		fast = fast->next;
		slow = slow->next;
	}

	auto tmp = slow->next;
	slow->next = slow->next->next;
	tmp->next = nullptr;
	return head;
}
