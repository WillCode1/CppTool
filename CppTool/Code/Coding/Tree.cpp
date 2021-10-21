#include "Tree.h"

// ¶þ²æÊ÷²ãÐò±éÀú£ºbfs
vector<int> TreeNode::levelOrder(TreeNode* root)
{
	vector<int> res;
	if (root == nullptr) {
		return res;
	}
	queue<TreeNode*> que;
	que.push(root);
	bfs(que, res);
	return res;
}

void TreeNode::bfs(queue<TreeNode*>& que, vector<int>& res)
{
	while (que.size() > 0) {
		auto node = que.front();
		que.pop();
		res.push_back(node->val);
		if (node->left) {
			que.push(node->left);
		}
		if (node->right) {
			que.push(node->right);
		}
	}
}
