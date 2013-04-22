#include <memory>
#include "World.hpp"
#include <iostream>

class Node
{
public:
	Node();
	Node(int x, int y, World world, std::shared_ptr<Node> _parent = 0);
	~Node(void);

	//return  G + H
	float getF();

	//Distance depuis la node actuelle jusqu'� la prochaine
	int manHattanDistance(std::shared_ptr<Node> nodeEnd);	

	//V�rifie si this est proche de goalNode
	bool isClosed(std::shared_ptr<Node> goalNode);

	//Position
	int m_x, m_y;

	//ID
	int m_id;

	//Pointeur vers le Node parent
	std::shared_ptr<Node> parent;

	//Cout du d�placement
	float G;

	//manHattanDistance : distance entre la node et lanode d'arriv�e
	int H;

	World m_world;
};

