#include "QuadTreeNodeWithOBB.h"
#include "BimGeoAlgorithmTool.h"

template <typename T>
QuadTreeNodeWithOBB<T>::QuadTreeNodeWithOBB(const double& dLeftX, const double& dLeftY, const double& dWidth, const double& dHeight,
	const int nLevel, const int nMaxLevel, const QuadType eQuadType, QuadTreeNodeWithOBB* pParent) :
	m_dX(dLeftX)
	, m_dY(dLeftY)
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
QuadTreeNodeWithOBB<T>::~QuadTreeNodeWithOBB()
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
bool QuadTreeNodeWithOBB<T>::IsIntersect(const double& dCenterX, const double& dCenterY, const double& dW, const double& dH, const double& dAngle, const T *object) const
{
	if (BimGeoAlgorithmTool::IsOutWithOBB(gp_Pnt2d(dCenterX, dCenterY), dW, dH, dAngle,
		gp_Pnt2d(object->dCenterX, object->dCenterY), object->dW, object->dH, object->dAngle))
	{
		return false;
	}

	return true;
}

template<typename T>
inline bool QuadTreeNodeWithOBB<T>::IsIntersect(const double& dCenterX, const double& dCenterY, const double& dW, const double& dH, const double& dAngle, const QuadTreeNodeWithOBB<T>* quadTreeNode) const
{
	Bnd_Box box = BimGeoAlgorithmTool::OBBToAABB(gp_Pnt2d(dCenterX, dCenterY), dW, dH, dAngle);
	if (box.CornerMax().X() < quadTreeNode->m_dX
		|| quadTreeNode->m_dX + quadTreeNode->m_dWidth < box.CornerMin().X()
		|| box.CornerMax().Y() < quadTreeNode->m_dY
		|| quadTreeNode->m_dY + quadTreeNode->m_dHeight < box.CornerMin().Y())
	{
		return false;
	}

	return true;
}

template <typename T>
bool QuadTreeNodeWithOBB<T>::IsIntersect(const double& dLeftX, const double& dLeftY, const double& dW, const double& dH, const QuadTreeNodeWithOBB<T> *quadTreeNode) const
{
	if (dLeftX + dW < quadTreeNode->m_dX
		|| quadTreeNode->m_dX + quadTreeNode->m_dWidth < dLeftX
		|| dLeftY + dH < quadTreeNode->m_dY
		|| quadTreeNode->m_dY + quadTreeNode->m_dHeight < dLeftY)
	{
		return false;
	}

	return true;
}

template <typename T>
void QuadTreeNodeWithOBB<T>::InsertObject(T *object)
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
			m_upRightNode = new QuadTreeNodeWithOBB(m_dX + m_dWidth / 2, m_dY, m_dWidth / 2, m_dHeight / 2, m_nlevel + 1, m_nMaxLevel, UP_RIGHT, this);//如果没有子节点就创建子节点，parent节点是当前节点
		}

		m_upRightNode->InsertObject(object);
		return;
	}
	else if (IsContain(m_dX, m_dY, m_dWidth / 2, m_dHeight / 2, object))
	{
		if (!m_upLeftNode)
		{
			m_upLeftNode = new QuadTreeNodeWithOBB(m_dX, m_dY, m_dWidth / 2, m_dHeight / 2, m_nlevel + 1, m_nMaxLevel, UP_LEFT, this);
		}

		m_upLeftNode->InsertObject(object);
		return;
	}
	else if (IsContain(m_dX, m_dY + m_dHeight / 2, m_dWidth / 2, m_dHeight / 2, object))
	{
		if (!m_bottomLeftNode)
		{
			m_bottomLeftNode = new QuadTreeNodeWithOBB(m_dX, m_dY + m_dHeight / 2, m_dWidth / 2, m_dHeight / 2, m_nlevel + 1, m_nMaxLevel, BOTTOM_LEFT, this);
		}
			
		m_bottomLeftNode->InsertObject(object);
		return;
	}
	else if (IsContain(m_dX + m_dWidth / 2, m_dY + m_dHeight / 2, m_dWidth / 2, m_dHeight / 2, object))
	{
		if (!m_bottomRightNode)
		{
			m_bottomRightNode = new QuadTreeNodeWithOBB(m_dX + m_dWidth / 2, m_dY + m_dHeight / 2, m_dWidth / 2, m_dHeight / 2, m_nlevel + 1, m_nMaxLevel, BOTTOM_RIGHT, this);
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
std::list<T *> QuadTreeNodeWithOBB<T>::GetObjectsAt(const double& dCenterX, const double& dCenterY, const double& dW, const double& dH, const double& dAngle)
{
	std::list<T *> resObjects;
	//如果当前节点完全被包含，把当前节点存的对象放到列表末尾,空链表也行
	if (IsIntersect(dCenterX, dCenterY, dW, dH, dAngle, this))
	{
		typename std::list<T *>::iterator itObject = m_objects.begin();
		for (; itObject != m_objects.end(); ++itObject)
		{
			if (IsIntersect(dCenterX, dCenterY, dW, dH, dAngle, *itObject))
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
		std::list<T *> upRightChild = m_upRightNode->GetObjectsAt(dCenterX, dCenterY, dW, dH, dAngle);
		resObjects.insert(resObjects.end(), upRightChild.begin(), upRightChild.end());
	}
	if (m_upLeftNode)
	{
		std::list<T *> upLeftChild = m_upLeftNode->GetObjectsAt(dCenterX, dCenterY, dW, dH, dAngle);
		resObjects.insert(resObjects.end(), upLeftChild.begin(), upLeftChild.end());
	}
	if (m_bottomLeftNode)
	{
		std::list<T *> bottomLeftChild = m_bottomLeftNode->GetObjectsAt(dCenterX, dCenterY, dW, dH, dAngle);
		resObjects.insert(resObjects.end(), bottomLeftChild.begin(), bottomLeftChild.end());
	}
	if (m_bottomRightNode)
	{
		std::list<T *> bottomRightChild = m_bottomRightNode->GetObjectsAt(dCenterX, dCenterY, dW, dH, dAngle);
		resObjects.insert(resObjects.end(), bottomRightChild.begin(), bottomRightChild.end());
	}

	return resObjects;
}

template <typename T>
void QuadTreeNodeWithOBB<T>::RemoveObjectsAt(const double& dLeftX, const double& dLeftY, const double& dW, const double& dH)
{
	//如果本层节点被包含则删除本层节点的对象
	//这个判断主要是对根节点起作用，其他子节点实际在上层都做了判断
	if (IsIntersect(dLeftX, dLeftY, dW, dH, this))
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
	if (m_upRightNode && IsContain(dLeftX, dLeftY, dW, dH, m_upRightNode))
	{
		m_upRightNode->RemoveObjectsAt(dLeftX, dLeftY, dW, dH);
		delete m_upRightNode;
		m_upRightNode = nullptr;

	}
	if (m_upLeftNode && IsContain(dLeftX, dLeftY, dW, dH, m_upLeftNode))
	{
		m_upLeftNode->RemoveObjectsAt(dLeftX, dLeftY, dW, dH);
		delete m_upLeftNode;
		m_upLeftNode = nullptr;

	}
	if (m_bottomLeftNode && IsContain(dLeftX, dLeftY, dW, dH, m_bottomLeftNode))
	{
		m_bottomLeftNode->RemoveObjectsAt(dLeftX, dLeftY, dW, dH);
		delete m_bottomLeftNode;
		m_bottomLeftNode = nullptr;

	}
	if (m_bottomRightNode && IsContain(dLeftX, dLeftY, dW, dH, m_bottomRightNode))
	{
		m_bottomRightNode->RemoveObjectsAt(dLeftX, dLeftY, dW, dH);
		delete m_bottomRightNode;
		m_bottomRightNode = nullptr;
	}
}

template<typename T>
inline bool QuadTreeNodeWithOBB<T>::IsContain(const double& dLeftX, const double& dLeftY, const double& dW, const double& dH, const T* object) const
{
	Bnd_Box box = BimGeoAlgorithmTool::OBBToAABB(gp_Pnt2d(object->dCenterX, object->dCenterY),
		object->dW, object->dH, object->dAngle);

	if (BimGeoAlgorithmTool::lessEqualThan(dLeftX, box.CornerMin().X())
		&& BimGeoAlgorithmTool::lessEqualThan(box.CornerMax().X(), dLeftX + dW)
		&& BimGeoAlgorithmTool::lessEqualThan(dLeftY, box.CornerMin().Y())
		&& BimGeoAlgorithmTool::lessEqualThan(box.CornerMax().Y(), dLeftY + dH))
	{
		return true;
	}

	return false;

}

template<typename T>
inline bool QuadTreeNodeWithOBB<T>::IsContain(const double& dLeftX, const double& dLeftY, const double& dW, const double& dH, const QuadTreeNodeWithOBB<T>* quadTreeNode) const
{
	if (quadTreeNode->m_dX >= dLeftX
		&& quadTreeNode->m_dX + quadTreeNode->m_dWidth <= dLeftX + dW
		&& quadTreeNode->m_dY >= dLeftY
		&& quadTreeNode->m_dY + quadTreeNode->m_dHeight <= dLeftY + dH)
	{
		return true;
	}

	return false;

}
