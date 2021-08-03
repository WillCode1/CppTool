#include "QuadTreeNode.h"
#include "BimGeoAlgorithmTool.h"

template <typename T>
QuadTreeNode<T>::QuadTreeNode(const double& dX, const double& dY, const double& dWidth, const double& dHeight,
	const int nLevel, const int nMaxLevel, const QuadType eQuadType, QuadTreeNode* pParent) :
	m_dX(dX)
	, m_dY(dY)
	, m_dWidth(dWidth)
	, m_dHeight(dHeight)
	, m_nlevel(nLevel)
	, m_nMaxLevel(nMaxLevel)
	, m_eQuadType(eQuadType)
	, m_Pparent(pParent)
	, m_upRightNode(nullptr)
	, m_upLeftNode(nullptr)
	, m_bottomLeftNode(nullptr)
	, m_bottomRightNode(nullptr)
{
}

template <typename T>
QuadTreeNode<T>::~QuadTreeNode()
{
	if (nullptr != m_upRightNode)
	{
		delete m_upRightNode;
		m_upRightNode = nullptr;
	}

	if (nullptr != m_upLeftNode)
	{
		delete m_upLeftNode;
		m_upLeftNode = nullptr;
	}

	if (nullptr != m_bottomLeftNode)
	{
		delete m_bottomLeftNode;
		m_bottomLeftNode = nullptr;
	}

	if (nullptr != m_bottomRightNode)
	{
		delete m_bottomRightNode;
		m_bottomRightNode = nullptr;
	}

	if (m_nlevel == m_nMaxLevel)
	{
		return;
	}

	//如果不是叶子节点，就销毁子节点
	m_Pparent = nullptr;
}

template <typename T>
bool QuadTreeNode<T>::IsIntersect(const double& dX, const double& dY, const double& dW, const double& dH, const T *object) const
{
	if (BimGeoAlgorithmTool::lessThan(dX + dW, object->dX)
		|| BimGeoAlgorithmTool::lessThan(object->dX + object->dW, dX)
		|| BimGeoAlgorithmTool::lessThan(dY + dH, object->dY)
		|| BimGeoAlgorithmTool::lessThan(object->dY + object->dH, dY))
	{
		return false;
	}

	return true;
}

template <typename T>
bool QuadTreeNode<T>::IsIntersect(const double& dX, const double& dY, const double& dW, const double& dH, const QuadTreeNode<T> *quadTreeNode) const
{
	if (dX + dW < quadTreeNode->m_dX
		|| quadTreeNode->m_dX + quadTreeNode->m_dWidth < dX
		|| dY + dH <quadTreeNode->m_dY
		|| quadTreeNode->m_dY + quadTreeNode->m_dHeight < dY)
	{
		return false;
	}

	return true;
}

template <typename T>
void QuadTreeNode<T>::InsertObject(T *object)
{
	//如果是叶子节点，则存在叶子节点
	if (m_nlevel == m_nMaxLevel)
	{
		m_objects.emplace_back(object);

		return;
	}

	//非叶子节点，如果下层节点可以包含该对象，则递归构建子节点并插入对象,边构建边插入
	if (IsContain(m_dX + m_dWidth / 2, m_dY, m_dWidth / 2, m_dHeight / 2, object))
	{
		if (!m_upRightNode) //避免重复创建覆盖掉原来的节点
		{
			m_upRightNode = new QuadTreeNode(m_dX + m_dWidth / 2, m_dY, m_dWidth / 2, m_dHeight / 2, m_nlevel + 1, m_nMaxLevel, UP_RIGHT, this);//如果没有子节点就创建子节点，parent节点是当前节点
		}

		m_upRightNode->InsertObject(object);
		return;
	}
	else if (IsContain(m_dX, m_dY, m_dWidth / 2, m_dHeight / 2, object))
	{
		if (!m_upLeftNode)
		{
			m_upLeftNode = new QuadTreeNode(m_dX, m_dY, m_dWidth / 2, m_dHeight / 2, m_nlevel + 1, m_nMaxLevel, UP_LEFT, this);
		}

		m_upLeftNode->InsertObject(object);
		return;
	}
	else if (IsContain(m_dX, m_dY + m_dHeight / 2, m_dWidth / 2, m_dHeight / 2, object))
	{
		if (!m_bottomLeftNode)
		{
			m_bottomLeftNode = new QuadTreeNode(m_dX, m_dY + m_dHeight / 2, m_dWidth / 2, m_dHeight / 2, m_nlevel + 1, m_nMaxLevel, BOTTOM_LEFT, this);
		}
			
		m_bottomLeftNode->InsertObject(object);
		return;
	}
	else if (IsContain(m_dX + m_dWidth / 2, m_dY + m_dHeight / 2, m_dWidth / 2, m_dHeight / 2, object))
	{
		if (!m_bottomRightNode)
		{
			m_bottomRightNode = new QuadTreeNode(m_dX + m_dWidth / 2, m_dY + m_dHeight / 2, m_dWidth / 2, m_dHeight / 2, m_nlevel + 1, m_nMaxLevel, BOTTOM_RIGHT, this);
		}

		m_bottomRightNode->InsertObject(object);
		return;
	}
	//下层节点不能完全包含改对象，则插入到当前非叶子节点
	//这个判断也可以省去
	if (IsContain(m_dX, m_dY, m_dWidth, m_dHeight, object))
	{
		m_objects.emplace_back(object);
	}
}

template <typename T>
std::list<T *> QuadTreeNode<T>::GetObjectsAt(const double& dX, const double& dY, const double& dW, const double& dH)
{
	std::list<T *> resObjects;
	//如果当前节点完全被包含，把当前节点存的对象放到列表末尾,空链表也行
	if (IsIntersect(dX, dY, dW, dH, this))
	{
		typename std::list<T *>::iterator itObject = m_objects.begin();
		for (; itObject != m_objects.end(); ++itObject)
		{
			if (IsIntersect(dX, dY, dW, dH, *itObject))
			{
				resObjects.emplace_back(*itObject);
			}
		}

		//最后一层
		if (m_nlevel == m_nMaxLevel)
		{
			return resObjects;
		}
	}
	else
	{
		return resObjects;
	}

	//如果有下层节点就把下层节点包含的对象加进来
	if (m_upRightNode)
	{
		std::list<T *> upRightChild = m_upRightNode->GetObjectsAt(dX, dY, dW, dH);
		resObjects.insert(resObjects.end(), upRightChild.begin(), upRightChild.end());
	}
	if (m_upLeftNode)
	{
		std::list<T *> upLeftChild = m_upLeftNode->GetObjectsAt(dX, dY, dW, dH);
		resObjects.insert(resObjects.end(), upLeftChild.begin(), upLeftChild.end());
	}
	if (m_bottomLeftNode)
	{
		std::list<T *> bottomLeftChild = m_bottomLeftNode->GetObjectsAt(dX, dY, dW, dH);
		resObjects.insert(resObjects.end(), bottomLeftChild.begin(), bottomLeftChild.end());
	}
	if (m_bottomRightNode)
	{
		std::list<T *> bottomRightChild = m_bottomRightNode->GetObjectsAt(dX, dY, dW, dH);
		resObjects.insert(resObjects.end(), bottomRightChild.begin(), bottomRightChild.end());
	}

	return resObjects;
}

template <typename T>
void QuadTreeNode<T>::RemoveObjectsAt(const double& dX, const double& dY, const double& dW, const double& dH)
{
	//如果本层节点被包含则删除本层节点的对象
	//这个判断主要是对根节点起作用，其他子节点实际在上层都做了判断
	if (IsIntersect(dX, dY, dW, dH, this))
	{
		//清除本节点层的对象
		m_objects.clear();
		//最后一层
		if (m_nlevel == m_nMaxLevel)
		{
			return;
		}

	}
	//如果有子节点且被包含就销毁子节点，注意别产生野指针
	//其实只要上层被包含了，下层肯定被包含，代码还需改进
	if (m_upRightNode && IsContain(dX, dY, dW, dH, m_upRightNode))
	{
		m_upRightNode->RemoveObjectsAt(dX, dY, dW, dH);
		delete m_upRightNode;
		m_upRightNode = nullptr;

	}
	if (m_upLeftNode && IsContain(dX, dY, dW, dH, m_upLeftNode))
	{
		m_upLeftNode->RemoveObjectsAt(dX, dY, dW, dH);
		delete m_upLeftNode;
		m_upLeftNode = nullptr;

	}
	if (m_bottomLeftNode && IsContain(dX, dY, dW, dH, m_bottomLeftNode))
	{
		m_bottomLeftNode->RemoveObjectsAt(dX, dY, dW, dH);
		delete m_bottomLeftNode;
		m_bottomLeftNode = nullptr;

	}
	if (m_bottomRightNode && IsContain(dX, dY, dW, dH, m_bottomRightNode))
	{
		m_bottomRightNode->RemoveObjectsAt(dX, dY, dW, dH);
		delete m_bottomRightNode;
		m_bottomRightNode = nullptr;
	}
}

template<typename T>
inline bool QuadTreeNode<T>::IsContain(const double& dX, const double& dY, const double& dW, const double& dH, const T* object) const
{
	if (BimGeoAlgorithmTool::lessEqualThan(dX, object->dX)
		&& BimGeoAlgorithmTool::lessEqualThan(object->dX + object->dW, dX + dW)
		&& BimGeoAlgorithmTool::lessEqualThan(dY, object->dY)
		&& BimGeoAlgorithmTool::lessEqualThan(object->dY + object->dH, dY + dH))
	{
		return true;
	}

	return false;

}

template<typename T>
inline bool QuadTreeNode<T>::IsContain(const double& dX, const double& dY, const double& dW, const double& dH, const QuadTreeNode<T>* quadTreeNode) const
{
	if (quadTreeNode->m_dX >= dX
		&& quadTreeNode->m_dX + quadTreeNode->m_dWidth <= dX + dW
		&& quadTreeNode->m_dY >= dY
		&& quadTreeNode->m_dY + quadTreeNode->m_dHeight <= dY + dH)
	{
		return true;
	}

	return false;

}
