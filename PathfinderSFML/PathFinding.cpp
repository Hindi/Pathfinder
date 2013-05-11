#include "PathFinding.h"
#include "World.hpp"

#define ABS(x) (((x) < 0) ? -(x) : (x))

PathFinding::PathFinding(World world):
	m_world(world)
{
	int id;
	for(int i(0); i <= world.worldLength; i += m_world.step)
		for(int j(0); j <= world.worldWidth; j +=  m_world.step)
		{
			int id = j*m_world.worldSize + i;
			if(!world.checkObstacleId(id))
			{
				m_grille[id] = Node(i, j, m_world);

				sf::CircleShape shape( m_world.step / 4);
				shape.setFillColor(sf::Color::Black);
				shape.setPosition(i,j);
				m_shapes.push_back(shape);
			}
		}
}


PathFinding::~PathFinding()
{

}

//Sert a lancer la recherche, fait les initialisations
void PathFinding::findPath(Vecteur start, Vecteur goal)
{	
	//Si le départ est au même endroit que l'arrivée ou sur un obstacle
	if((start.x == goal.x && start.y == goal.y) || m_world.checkObstacle(Vecteur(goal.x, goal.y)))
	{
		//On met l'arrivée dans la liste des positions
		m_resultPath.push_back(Vecteur(goal.x, goal.y));
		return;
	}
	
	//Clear
	m_openList.clear();
	m_closedList.clear();
	m_resultPath.clear();

	int startId = start.y*m_world.worldSize + start.x;
	int goalId = goal.y*m_world.worldSize + goal.x;

	//On initialise les noeuds de départ et d'arrivée
	m_currentNode = new Node(m_grille[startId]);
	m_goalNode = m_grille[goalId];

	//On rajoute la node de départ à la liste des nodes disponibles
	m_closedList.insert(startId);

	//On check les nodes alentours
	this->checkNeighbourNode();
}

//Rajoute une node à la liste des nodes disponibles pour le chemin
void PathFinding::addToOpenList(float x, float y, float moveCost,  Node* parent)
{
	int id = y * m_world.worldSize + x;
	std::unordered_map<int, Node>::iterator existMap = m_grille.find(id);
	if(existMap == m_grille.end())
		return;

	//On s'arrête là si les calculs ont déjà été fait pour cette node
	std::unordered_set<int>::iterator existOpen = m_openList.find(id);
	if(existOpen != m_openList.end())
		return;
	std::unordered_set<int>::iterator existClose = m_closedList.find(id);
	if(existClose != m_closedList.end())
		return;

	m_grille[id].G = moveCost;
	m_grille[id].manHattanDistance(m_goalNode);
	m_grille[id].parent = parent;
	m_openList.insert(id);
	
    sf::CircleShape shape( m_world.step / 4);
    shape.setFillColor(sf::Color::Green);
	shape.setPosition(x, y);
	m_shapes.push_back(shape);
}

//Lance les calculs sur les nodes alentours, vérifie si l'on est arrivé
void PathFinding::checkNeighbourNode()
{
	//Si l'objectif n'est pas proche
	if(!m_currentNode->isClosed(m_goalNode))
	{	
		this->addToOpenList(m_currentNode->m_x +m_world.step, m_currentNode->m_y, 1, m_currentNode);
		this->addToOpenList(m_currentNode->m_x, m_currentNode->m_y +m_world.step, 1, m_currentNode);
		this->addToOpenList(m_currentNode->m_x +m_world.step, m_currentNode->m_y +m_world.step, 1.4f, m_currentNode);
		this->addToOpenList(m_currentNode->m_x -m_world.step, m_currentNode->m_y -m_world.step, 1.4f, m_currentNode);
		this->addToOpenList(m_currentNode->m_x -m_world.step, m_currentNode->m_y, 1, m_currentNode);
		this->addToOpenList(m_currentNode->m_x -m_world.step, m_currentNode->m_y +m_world.step, 1.4f, m_currentNode);
		this->addToOpenList(m_currentNode->m_x +m_world.step, m_currentNode->m_y -m_world.step, 1.4f, m_currentNode);
		this->addToOpenList(m_currentNode->m_x, m_currentNode->m_y -m_world.step, 1, m_currentNode);
		
		std::cout << m_openList.size() << std::endl;
		//Si m_openList est vide, cela signifie que la position demandé n'est pas accessible
		if(m_openList.empty())
			return;
		else
		{
			//On regarde la prochaine node sur laquelle on va fixer nos recherches
			//On choisit la node dans m_openList avec le F minimum
			float Fvalue = 99999;
			int eraseIt;
			std::unordered_set<int>::iterator it = m_openList.begin();
			for(; it != m_openList.end(); it++)
				if(Fvalue > m_grille[(*it)].getF())
				{
					Fvalue = m_grille[(*it)].getF();
					eraseIt = (*it);
				}
			m_currentNode = new Node(m_grille[eraseIt]);
			//Et on la rajoute à la liste des nodes potentiellement sur le chemin final
			m_closedList.insert(eraseIt);
			//On l'enlève de la liste des nodes disponibles
			m_openList.erase(eraseIt);

			sf::CircleShape shape( m_world.step / 4);
			shape.setFillColor(sf::Color::Red);
			shape.setPosition(m_currentNode->m_x, m_currentNode->m_y);
			m_shapes.push_back(shape);

			//On check les nodes alentours : récursif
			this->checkNeighbourNode();
		}
	}
	//Si on est proche de l'objectif
	else
	{
		//On récupère le parent des nodes une à une pour remonter jusqu'au départ
		m_goalNode.parent = m_currentNode;

		for(Node* getPath = new Node(m_goalNode); getPath->parent != NULL; getPath = getPath->parent)
		{
			std::cout << getPath->m_x << " " << getPath->m_y << std::endl;
			m_resultPath.push_back(Vecteur(getPath->m_x, getPath->m_y));
			sf::CircleShape shape( m_world.step / 4);
			shape.setFillColor(sf::Color::Blue);
			shape.setPosition(getPath->m_x, getPath->m_y);
			m_shapes.push_back(shape);
		}
		delete m_currentNode;
		return;
	}
}

//Renvoie le chemin vers l'objectif
std::vector<Vecteur> PathFinding::getPath()
{
	return m_resultPath;
}

void PathFinding::draw(sf::RenderWindow &window)
{
	std::vector<sf::CircleShape>::iterator it = m_shapes.begin();
	for(; it != m_shapes.end(); it++)
		window.draw(*it);
	std::vector<sf::Text>::iterator itt = m_text.begin();
	for(; itt != m_text.end(); itt++)
		window.draw(*itt);
}

bool PathFinding::lineOfSight(std::shared_ptr<Node> startNode, std::shared_ptr<Node> goalNode)
{
	int x = startNode->m_x;
	int y = startNode->m_y;
	int gx = goalNode->m_x;
	int gy = goalNode->m_y;

	int dx = gx - x;
	int dy = gy - y;
	int sx = 5;
	if(dx < 0)
	{
		sx = -sx;
		dx = - dx;
	}

	if(dy < 0)
		dy = - dy;

	float a, b;

	if(dx== 0 && dy ==0)
		return true;

	if(dy != 0)
		a = (gx - x)/(gy - y);
	else
		a = 0;
	b = y - a*x;
	
	std::cout << x << "," << y << " " << gx << "," << gy << std::endl;
		
	while(dy > 10 && dx > 10)
	{
		y = b + a*x;
		
		if(m_world.checkObstacle(Vecteur(x,y)))
		{
			std::cout << x << " "  << y << std::endl;
			return false;
		}
		dx = ABS(gx - x);
		dy = ABS(gy - y);
		x += sx;
	}
	/*
		*/
	
	return true;
	/*

	int x = startNode->m_x;
	int y = startNode->m_y;
	int gx = goalNode->m_x;
	int gy = goalNode->m_y;

	int dx = gx - x;
	int dy = gy - y;

	int sx = m_world.step;
	int sy = m_world.step;

	int f = 0;

	if(dx < 0)
	{
		sx = -sx;
		dx = - dx;
	}

	if(dy < 0)
	{
		sy = -sy;
		dy = - dy;

	if(dx >= dy)
		while(x != gx)
		{
			f += dy;
			if(f >= dx)
			{
				if(m_world.checkObstacle(Vecteur(x + (sx - 1)/2,y + (sy - 1)/2)))
					return false;
				y += sy;
				f -= dx;
			}
			if(f != 0 && m_world.checkObstacle(Vecteur(x + (sx - 1)/2, y + (sy - 1)/2)))
				return false;
			if(dy == 0 && m_world.checkObstacle(Vecteur(x + (sx - 1)/2,y)) && m_world.checkObstacle(Vecteur(x + (sx - 1)/2, y - 1)) )
				return false;
			x += sx;
		}
	else
		while(y != gy)
		{
			f += dx;
			if(f >= dy)
			{
				if(m_world.checkObstacle(Vecteur(x + (sx - 1)/2,y + (sy - 1)/2)))
					return false;
				x += sx;
				f -= dy;
			}
			if(f != 0 && m_world.checkObstacle(Vecteur(x + (sx - 1)/2, y + (sy - 1)/2)))
				return false;
			if(dx == 0 && m_world.checkObstacle(Vecteur(x,y + (sy - 1)/2)) && m_world.checkObstacle(Vecteur(x -1, y + (sy - 1)/2)) )
				return false;
			y += sy;
		}
		*/
		return true;
}