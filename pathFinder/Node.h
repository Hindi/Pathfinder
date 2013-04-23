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

	//Distance depuis la node actuelle jusqu'à la prochaine
	int manHattanDistance(Node nodeEnd);	

	//Vérifie si this est proche de goalNode
	bool isClosed(Node goalNode);

	Node& operator=(const Node &node);

	//Position
	float m_x, m_y;

	//ID
	float m_id;

	//Pointeur vers le Node parent
	Node* parent;

	//Cout du déplacement
	float G;

	//manHattanDistance : distance entre la node et lanode d'arrivée
	int H;

	World m_world;
};

