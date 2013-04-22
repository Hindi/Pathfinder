#include "PathFinding.h"
#include "World.hpp"

#define ABS(x) (((x) < 0) ? -(x) : (x))

PathFinding::PathFinding(World world):
	m_start(0,0),
	m_goal(0,0),
	m_world(world)
{
	for(int i(0); i <= world.worldLength; i += m_world.step)
		for(int j(0); j <= world.worldWidth; j +=  m_world.step)
		{
			if(!world.checkObstacle(Vecteur(i,j)))
			{
				m_grille.push_back(std::shared_ptr<Node>(new Node(i, j, m_world)));
				sf::CircleShape shape( m_world.step / 4);
				shape.setFillColor(sf::Color::Black);
				shape.setPosition(i,j);
				m_shapes.push_back(shape);
			}
		}
}


PathFinding::~PathFinding(void)
{

}

//Sert a lancer la recherche, fait les initialisations
void PathFinding::findPath(Vecteur start, Vecteur goal)
{
	m_start = start;
	m_goal = goal;

	
	//Si le départ est au même endroit que l'arrivée ou sur un obstacle
	if((m_start.x == m_goal.x && m_start.y == m_goal.y) || m_world.checkObstacle(Vecteur(m_goal.x, m_goal.y)))
	{
		//On met l'arrivée dans la liste des positions
		m_resultPath.push_back(new Vecteur(m_goal.x, m_goal.y));
		return;
	}
	
	//Clear
	m_openList.clear();
	m_closedList.clear();
	m_resultPath.clear();

	int startId = start.y*m_world.worldSize + start.x;
	int goalId = goal.y*m_world.worldSize + goal.x;

	//On initialise les noeuds de départ et d'arrivée
	std::vector< std::shared_ptr<Node> >::iterator it = m_grille.begin();
	for(; it != m_grille.end(); it++)
	{
		if((*it)->m_id == startId)
		{
			(*it) = std::shared_ptr<Node>(new Node(m_start.x, m_start.y, m_world, NULL));
			m_currentNode = (*it);
		}
		else if((*it)->m_id == goalId)
		{
			(*it)->parent = (*it);
			m_goalNode = (*it);
		}
	}

	//On rajoute la node de départ à la liste des nodes disponibles
	m_closedList.push_back(m_currentNode);

	//On check les nodes alentours
	this->checkNeighbourNode();
}

//Rajoute une node à la liste des nodes disponibles pour le chemin
void PathFinding::addToOpenList(float x, float y, float moveCost,  std::shared_ptr<Node>  parent)
{
	//On vérifie les collisions avec les obstacles fixes
	if(m_world.checkObstacle(Vecteur(x, y)))
		return;

	int id = y * m_world.worldSize + x;

	//On s'arrête là si les calculs ont déjà été fait pour cette node
	std::vector< std::shared_ptr<Node> >::iterator it = m_openList.begin();
	for(; it != m_openList.end(); it++)
		if(id == (*it)->m_id)
			return;
	std::vector< std::shared_ptr<Node> >::iterator itt = m_closedList.begin();
	for(; itt != m_closedList.end(); itt++)
		if(id == (*itt)->m_id)
			return;

	std::vector< std::shared_ptr<Node> >::iterator lit = m_grille.begin();
	for(; lit != m_grille.end(); lit++)
		if(id == (*lit)->m_id)
		{
			(*lit)->parent = parent;
			(*lit)->G = moveCost;
			(*lit)->H = (*lit)->manHattanDistance(m_goalNode);
			m_openList.push_back((*lit));
		}

	
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

		//Si m_openList est vide, cela signifie que la position demandé n'est pas accessible
		if(m_openList.empty())
			return;
		else
		{
			//On regarde la prochaine node sur laquelle on va fixer nos recherches
			//On choisit la node dans m_openList avec le F minimum
			float Fvalue = 99999;
			std::vector< std::shared_ptr<Node> >::iterator eraseIt;
			std::vector< std::shared_ptr<Node> >::iterator it = m_openList.begin();
			for(; it != m_openList.end(); it++)
				if(Fvalue > (*it)->getF())
				{
					Fvalue = (*it)->getF();
					eraseIt = it;
				}
			m_currentNode = *eraseIt;
			//On l'enlève de la liste des nodes disponibles
			m_openList.erase(eraseIt);
			//Et on la rajoute à la liste des nodes potentiellement sur le chemin final
			m_closedList.push_back(m_currentNode);

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
		m_goalNode->parent = m_currentNode;
		std::shared_ptr<Node> lastVisibleNode = m_goalNode;
		std::shared_ptr<Node> savedNode = m_currentNode;
		std::shared_ptr<Node>  getPath;
		for(getPath = m_goalNode->parent; getPath != NULL; getPath = getPath->parent)
		{
			/*if(lineOfSight(lastVisibleNode, getPath))
			{
				savedNode = getPath;
			}
			else*/
			{
				savedNode = getPath;
				m_resultPath.push_back(new Vecteur(savedNode->m_x, savedNode->m_y));
				lastVisibleNode = savedNode;
				sf::CircleShape shape( m_world.step / 4);
				shape.setFillColor(sf::Color::Blue);
				shape.setPosition(savedNode->m_x, savedNode->m_y);
				m_shapes.push_back(shape);
			}
		}
		return;
	}
}

//Renvoie le chemin vers l'objectif
std::vector<Vecteur*> PathFinding::getPath()
{
	return m_resultPath;
}

void PathFinding::draw(sf::RenderWindow &window)
{
	std::vector<sf::CircleShape>::iterator it = m_shapes.begin();
	for(; it != m_shapes.end(); it++)
		window.draw(*it);
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