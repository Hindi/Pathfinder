#include <memory>
#include "World.hpp"

class Node
{
public:
	Node();
	Node(float x, float y, World world, Node* _parent = 0);
	~Node(void);

	//return  G + H
	float getF();

	//Distance depuis la node actuelle jusqu'� la prochaine
	int manHattanDistance(Node nodeEnd);	

	//V�rifie si this est proche de goalNode
	bool isClosed(Node goalNode);

	Node& operator=(const Node &node);

	//Position
	float m_x, m_y;

	//ID
	float m_id;

	//Pointeur vers le Node parent
	Node* parent;

	//Cout du d�placement
	float G;

	//manHattanDistance : distance entre la node et lanode d'arriv�e
	int H;

	World m_world;
};

