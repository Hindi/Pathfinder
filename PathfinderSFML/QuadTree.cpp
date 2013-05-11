#include "QuadTree.h"

BoundingBox::BoundingBox():
	m_position(Vecteur(0,0)),
	m_size(Vecteur(0,0))
{

}

BoundingBox::BoundingBox(Vecteur position, Vecteur size, int idSize):
	m_position(position),
	m_size(size),
	m_idSize(idSize)
{
}

bool BoundingBox::checkBoundingBox(Vecteur position)
{
	return (position.x > m_position.x && position.x < m_position.x + m_size.x && 
		position.y > m_position.y && position.y < m_position.y + m_size.y);
}

bool BoundingBox::checkBoundingBox(int id)
{
	Vecteur position;
	position.x = (int)(id/m_idSize);
	position.y = id - position.x*m_idSize;
	return checkBoundingBox(position);
}

QuadTree::QuadTree()
{

}

QuadTree::QuadTree(Vecteur worldSize, int idSize):
		m_worldSize(worldSize)
{
	int nombreZoneX(2), nombreZoneY(2);
	for(int i(0); i <= nombreZoneX; i++)
		for(int j(0); j <= nombreZoneY; j++)
		{
			BoundingBox box(Vecteur(i*(worldSize.x/nombreZoneX), j*(worldSize.y/nombreZoneY)), 
							Vecteur(worldSize.x / nombreZoneX, worldSize.y / nombreZoneY), idSize);
			m_boundingBox.push_back(box);
		}
}


QuadTree::~QuadTree(void)
{
}


void QuadTree::addObstacleId(int id)
{
	std::vector<BoundingBox>::iterator it(m_boundingBox.begin());
	for(; it != m_boundingBox.end(); it++)
		if(it->checkBoundingBox(id))
			it->m_obstaclesId.insert(id);
}



bool QuadTree::containsObstacleId(int id)
{
	std::vector<BoundingBox>::iterator it(m_boundingBox.begin());
	for(; it != m_boundingBox.end(); it++)
		if(it->checkBoundingBox(id))
		{
			std::unordered_set<int>::const_iterator itt = it->m_obstaclesId.find(id);
			return !(itt == it->m_obstaclesId.end());
		}
}
