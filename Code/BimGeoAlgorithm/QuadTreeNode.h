#pragma once

#include "BimGeoAlgorithmDef.h"
#include <list>

//四叉树类型枚举
enum QuadType
{
	ROOT,         //根
	UP_RIGHT,     //象限Ⅰ
	UP_LEFT,      //象限Ⅱ
	BOTTOM_LEFT,  //象限Ⅲ
	BOTTOM_RIGHT  //象限Ⅳ
};


/// @class QuadTreeNode
/// @brief object 的格式是 {double dX, dY, dW, dH}

template <typename T>
class QuadTreeNode
{

public:
	QuadTreeNode(const double& dX, const double& dY, const double& dWidth, const double& dHeight,
		const int nLevel, const int nMaxLevel, const QuadType eQuadType, QuadTreeNode* pParent);
	~QuadTreeNode();
public:
	void InsertObject(T *object); //插入对象
	std::list<T *> GetObjectsAt(const double& dX, const double& dY, const double& dW, const double& dH); //查询对象,获得一片区域里的对象链表，此处只考虑完全包含的
	void RemoveObjectsAt(const double& dX, const double& dY, const double& dW, const double& dH); //删除对象，删除一片区域里的对象和节点，此处只考虑完全包含的
private:
	bool IsContain(const double& dX, const double& dY, const double& dW, const double& dH, const T* object) const;
	bool IsContain(const double& dX, const double& dY, const double& dW, const double& dH, const QuadTreeNode<T>* quadTreeNode) const;
	bool IsIntersect(const double& dX, const double& dY, const double& dW, const double& dH, const T* object) const;
	bool IsIntersect(const double& dX, const double& dY, const double& dW, const double& dH, const QuadTreeNode<T>* quadTreeNode) const;
private:
	std::list<T *>		m_objects;				//	节点数据队列
	QuadTreeNode*		m_Pparent;				//	父节点
	QuadTreeNode*		m_upRightNode;			//	右上节点
	QuadTreeNode*		m_upLeftNode;			//	左上节点
	QuadTreeNode*		m_bottomLeftNode;		//	左下节点
	QuadTreeNode*		m_bottomRightNode;		//	右下节点
	double				m_dX;					//	左下x坐标
	double				m_dY;					//	左下y坐标
	double				m_dWidth;				//	宽度
	double				m_dHeight;				//	高度
	QuadType			m_eQuadType;			//	节点类型
	int					m_nlevel;				//	当前深度
	int					m_nMaxLevel;			//	最大深度
};

#include "QuadTreeNodeImp.hpp"
