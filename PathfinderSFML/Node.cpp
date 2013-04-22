#include "Node.h"

#define ABS(x) (((x) < 0) ? -(x) : (x))

Node::Node(void): 
	parent(NULL),
	G(0), 
	H(0)
{
}

Node::Node(int x, int y, World world, std::shared_ptr<Node> _parent):
	m_x(x), 
	m_y(y), 
	parent(_parent),
	m_id(y * world.worldSize + x), 
	G(0), 
	H(0),
	m_world(world)
{

}

Node::~Node(void)
{

}

float Node::getF()
{
	 return G + H;
}

bool Node::isClosed(std::shared_ptr<Node> goalNode)
{
	return (fabs((float)goalNode->m_x - m_x) <= m_world.step && fabs((float)goalNode->m_y - m_y) <= m_world.step);
}

//Distance depuis la node actuelle jusqu'� l'objectif
int Node::manHattanDistance(std::shared_ptr<Node> nodeEnd)	
{
	int x = (ABS((this->m_x - nodeEnd->m_x)) / m_world.step);
	int y = (ABS((this->m_y - nodeEnd->m_y)) / m_world.step);
	return x + y;
}