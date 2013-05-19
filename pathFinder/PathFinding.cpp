#include "PathFinding.h"
#include "World.hpp"

#define ABS(x) (((x) < 0) ? -(x) : (x))

/*
 * Pour utiliser le pathfinding il faut d'abord mettre à jour 
 * m_start et m_goal qui sont les vecteurs de départ et d'arrivée.
 * Il faut ensuite utiliser findpath() et finalement récupérer la
 * liste de vecteurs qui forment le chemin grâce à getPath()
 */

PathFinding::PathFinding(World world):
	m_world(world)
{
	//On créé la liste map de noeuds en stockant les ID de chaque position disponible tous les "world.step" millimètres
	//L'ID est calculé de la manière suivante : y * world.worldSize +x
	for(int i(0); i <= world.length; i += m_world.step)
		for(int j(0); j <= world.worldWidth; j +=  m_world.step)
			if(!world.checkObstacle(Vecteur(i,j)))
			{
				m_grille[j*m_world.worldSize + i] = Node(i, j, m_world);
				
				sf::CircleShape shape( m_world.step / 4);
				shape.setFillColor(sf::Color::Black);
				shape.setPosition(i,j);
				m_shapes.push_back(shape);
			}
			else
				m_staticObstacles.insert(j*m_world.worldSize + i);
}


PathFinding::~PathFinding()
{

}

//Sert a lancer la recherche, fait les initialisations
std::vector<Vecteur> PathFinding::findPath(Vecteur start, Vecteur goal)
{	
	//Si le départ est au même endroit que l'arrivée ou sur un obstacle
	if((start.x == goal.x && start.y == goal.y) || m_world.checkObstacle(Vecteur(goal.x, goal.y)))
	{
		//On met l'arrivée dans la liste des positions
		m_resultPath.push_back(Vecteur(start.x, start.y));
		return m_resultPath;
	}
	
	//Clear
	m_openList.clear();
	m_closedList.clear();
	m_resultPath.clear();

	int startId = start.y*m_world.worldSize + start.x;
	int goalId = goal.y*m_world.worldSize + goal.x;

	m_grille[startId] = Node(start.x, start.y, m_world);
	m_grille[goalId] = Node(goal.x, goal.y, m_world);

	//On initialise les noeuds de départ et d'arrivée
	//m_currentNode = new Node(m_grille[startId]);
	m_goalNode = m_grille[goalId];
	m_startNode = m_grille[startId];
	Vecteur pos = findCloseNode(start.x, start.y);
	m_currentNode = new Node(m_grille[pos.y*m_world.worldSize + pos.x]);
		
	//On rajoute la node de départ à la liste des nodes disponibles
	m_closedList.insert(startId);

	//On check les nodes alentours
	this->checkNeighbourNode();
	
	//L'algorithme permet d'obtenir le chemin à l'envers (parcourt des nodes parents)
	//On inverse donc l'ordre des Vecteur
	revertPath();
	//changeCoordinateSystem();
	m_enemyList.clear();
	smoothPath();
	return m_resultPath;
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
	//On s'arrête également si la nodes est occupée par un ennemis
	std::unordered_set<int>::iterator existEnemy = m_enemyList.find(id);
	if(existEnemy != m_enemyList.end())
		return;

	//Si c'est la première fois qu'on croise cette node, on fait les calculs
	m_grille[id].G = moveCost;
	m_grille[id].manHattanDistance(m_goalNode);
	m_grille[id].parent = parent;
	
	//On l'ajoute à openList pour la rendre disponible comme currentNode et 
	//Pour ne pas refaire les calculs dessus la prochaine fois
	m_openList.insert(id);
}

//Lance les calculs sur les nodes alentours, vérifie si l'on est arrivé
void PathFinding::checkNeighbourNode()
{
	//Si l'objectif n'est pas proche
	if(!m_currentNode->isClosed(m_goalNode))
	{	
		//On utilise addToOpenList() sur chaque node adjacente à currentNode en précisant le coût de déplacement
		//(10 horizontal et vertical, 14 diagonal)
		this->addToOpenList(m_currentNode->m_x +m_world.step, m_currentNode->m_y, 10, m_currentNode);
		this->addToOpenList(m_currentNode->m_x, m_currentNode->m_y +m_world.step, 10, m_currentNode);
		this->addToOpenList(m_currentNode->m_x +m_world.step, m_currentNode->m_y +m_world.step, 14, m_currentNode);
		this->addToOpenList(m_currentNode->m_x -m_world.step, m_currentNode->m_y -m_world.step, 14, m_currentNode);
		this->addToOpenList(m_currentNode->m_x -m_world.step, m_currentNode->m_y, 10, m_currentNode);
		this->addToOpenList(m_currentNode->m_x -m_world.step, m_currentNode->m_y +m_world.step, 14, m_currentNode);
		this->addToOpenList(m_currentNode->m_x +m_world.step, m_currentNode->m_y -m_world.step, 14, m_currentNode);
		this->addToOpenList(m_currentNode->m_x, m_currentNode->m_y -m_world.step, 10, m_currentNode);
		
		//Si m_openList est vide, cela signifie que la position demandé n'est pas accessible
		if(m_openList.empty())
			return;
		else
		{
			//On regarde la prochaine node sur laquelle on va fixer nos recherches
			//On choisit la node dans m_openList avec le F minimum
			float Fvalue = 999;
			int eraseIt;
			std::unordered_set<int>::iterator it = m_openList.begin();
			for(; it != m_openList.end(); it++)
			{
				int F = m_grille[(*it)].getF();
				if(Fvalue > F)
				{
					Fvalue = F;
					eraseIt = (*it);
				}
			}
			//On initialise la valeur de currentNode
			m_currentNode = new Node(m_grille[eraseIt]);
			//On la rajoute à la liste des nodes potentiellement sur le chemin final
			m_closedList.insert(eraseIt);
			//On l'enlève de la liste des nodes disponibles pour currentNode
			m_openList.erase(eraseIt);

			//On check les nodes alentours : récursif
			this->checkNeighbourNode();
		}
	}
	//Si on est proche de l'objectif
	else
	{
		//On récupère le parent des nodes une à une de la node d'arrivée jusqu'a la node de départ
		//PArcourir les nodes parents permet d'obtenir le chemin le plus court
		m_goalNode.parent = m_currentNode;
		for(Node* getPath = new Node(m_goalNode); getPath != NULL; getPath = getPath->parent)
		{
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

void PathFinding::addEnemyPosition(int x, int y, int rayon)
{
  m_enemyList.insert(y*m_world.step +x);
  Vecteur closePo(findCloseNode(x,y));
  x = closePo.x;
  y = closePo.y;
  std::cout << x << " " << y << std::endl;
  int range = rayon/m_world.step;
  int diagRange = pow(2*pow((double)m_world.step, 2),0.5)/m_world.step;

  for(int i(1); i <= range+1; i+=m_world.step)
  {
    m_enemyList.insert((i*m_world.step+y)*m_world.worldSize + x);
    m_enemyList.insert((-i*m_world.step+y)*m_world.worldSize + x);
    m_enemyList.insert(y*m_world.worldSize + x+i*m_world.step);
    m_enemyList.insert(y*m_world.worldSize + x-i*m_world.step);
	
	sf::CircleShape shape( m_world.step / 4);
	shape.setFillColor(sf::Color::Magenta);
	shape.setPosition(x, i*m_world.step+y);
	m_shapes.push_back(shape);
	sf::CircleShape shapee( m_world.step / 4);
	shapee.setFillColor(sf::Color::Magenta);
	shapee.setPosition(x, -i*m_world.step+y);
	m_shapes.push_back(shapee);
	sf::CircleShape shapeee( m_world.step / 4);
	shapeee.setFillColor(sf::Color::Magenta);
	shapeee.setPosition(x+i*m_world.step, y);
	m_shapes.push_back(shapeee);
	sf::CircleShape shapeeee( m_world.step / 4);
	shapeeee.setFillColor(sf::Color::Magenta);
	shapeeee.setPosition(x-i*m_world.step, y);
	m_shapes.push_back(shapeeee);
  }
  
  for(int i(1); i <= diagRange+1; i+=m_world.step)
  {
    m_enemyList.insert((i*m_world.step+y)*m_world.worldSize + x+i*m_world.step);
    m_enemyList.insert((-i*m_world.step+y)*m_world.worldSize + x+i*m_world.step);
    m_enemyList.insert((-i*m_world.step+y)*m_world.worldSize + x-i*m_world.step);
    m_enemyList.insert((i*m_world.step+y)*m_world.worldSize + x+i*m_world.step);

	sf::CircleShape shape( m_world.step / 4);
	shape.setFillColor(sf::Color::Magenta);
	shape.setPosition(x+i*m_world.step, i*m_world.step+y);
	m_shapes.push_back(shape);
	sf::CircleShape shapee( m_world.step / 4);
	shapee.setFillColor(sf::Color::Magenta);
	shapee.setPosition(x+i*m_world.step, -i*m_world.step+y);
	m_shapes.push_back(shapee);
	sf::CircleShape shapeee( m_world.step / 4);
	shapeee.setFillColor(sf::Color::Magenta);
	shapeee.setPosition(x-i*m_world.step, x+i*m_world.step);
	m_shapes.push_back(shapeee);
	sf::CircleShape shapeeee( m_world.step / 4);
	shapeeee.setFillColor(sf::Color::Magenta);
	shapeeee.setPosition(x-i*m_world.step, -i*m_world.step+y);
	m_shapes.push_back(shapeeee);
  }
}

bool PathFinding::lineOfSight(Vecteur start, Vecteur goal)
{
	int x = start.x;
	int y = start.y;
	int gx =  goal.x;
	int gy =  goal.y;
	int distance;

	int dx = ABS(gx - x);
	int dy = ABS(gy - y);

	float a, b;

	if(dx == 0 && dy ==0)
		return true;

	if(dy != 0)
	{
		a = dx/dy;
		b = y - a*x;
    
		std::cout << y << " = " << a <<"*" << x <<" + " << b << std::endl;

		std::unordered_set<int>::iterator it(m_staticObstacles.begin());
		for(; it != m_staticObstacles.end(); it++)
		{
			y = (*it)/m_world.worldSize;
			x = (*it) -y;
			distance =  ABS(-a*x -y +b ) / pow((int)pow(a,2) +1, 0.5);
			if(distance  < m_world.step)
			{
			std::cout << "distance" << " " << ABS(-a*x -y +b)/ pow((int)pow(a,2) +1, 0.5) << std::endl;
				return false;
			}
		}
	}
	
	else
	  distance = dx;
	
	
	return true;
}

void PathFinding::smoothPath()
{
    Vecteur startPosition(*m_resultPath.begin());
    Vecteur lastVisiblePosition(*m_resultPath.begin());
    std::vector<Vecteur> tempPath;
    tempPath.push_back(startPosition);
    std::vector<Vecteur>::iterator it(m_resultPath.begin());
    for(; it != m_resultPath.end(); it++)
    {
      if(!lineOfSight(startPosition, *it))
	  {
		tempPath.push_back(lastVisiblePosition);
		std::cout << "NEW NODE" << std::endl;
		sf::CircleShape shape( m_world.step / 4);
		shape.setFillColor(sf::Color::Green);
		shape.setPosition(lastVisiblePosition.x, lastVisiblePosition.y);
		m_shapes.push_back(shape);
	  }
      else
		lastVisiblePosition = *it;
    }
    m_resultPath = tempPath;
}

Vecteur PathFinding::findCloseNode(Vecteur pos)
{
	//On divise et on stocke dans un int pour arrondir
	pos.x = pos.x / m_world.step;
	pos.y = pos.y / m_world.step;
	return pos;
}

Vecteur PathFinding::findCloseNode(int x, int y)
{
	//On divise et on stocke dans un int pour arrondir
	x = x / m_world.step;
	y = y / m_world.step;
	return Vecteur(x*m_world.step,y*m_world.step);
}

//Inverse l'ordre des coordonnées dans le chemin obtenu
void PathFinding::revertPath()
{
	//On utilise un conteneur temporaire pour inverser l'ordre des Vecteur
	//On parcourt m_resultPath en sens inverse et on push_back dans tempPath
	std::vector<Vecteur> tempPath;
	std::vector<Vecteur>::reverse_iterator it(m_resultPath.rbegin());
	for(;  it != m_resultPath.rend(); it++)
	  tempPath.push_back(*it);
	m_resultPath = tempPath;
}

void PathFinding::changeCoordinateSystem()
{
    std::vector<Vecteur>::reverse_iterator it(m_resultPath.rbegin());
	for(;  it != m_resultPath.rend(); it++)
	{
	  (*it).x = (*it).x - m_world.worldWidth/2;
	  (*it).y = m_world.length - (*it).y;
	}
}

void PathFinding::draw(sf::RenderWindow &window)
{
	std::vector<sf::CircleShape>::iterator it = m_shapes.begin();
	for(; it != m_shapes.end(); it++)
		window.draw(*it);
}