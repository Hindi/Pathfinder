#include <memory>
#include "World.hpp"
#include <iostream>

class Node
{
public:
	Node();
	Node(int x, int y, int id, World world, Node* _parent = 0);
	~Node(void);

	//return  G + H
	float getF();

	//Distance depuis la node actuelle jusqu'à la prochaine
	void manHattanDistance(Node nodeEnd);	

	//Vérifie si this est proche de goalNode
	bool isClosed(Node goalNode);

	Node& operator=(const Node &node);

	//Position
	int m_x, m_y;

	//ID
	int m_id;

	//Pointeur vers le Node parent
	Node* parent;

	//Cout du déplacement
	float G;

	//manHattanDistance : distance entre la node et lanode d'arrivée
	int H;

	World m_world;
};

