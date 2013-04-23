#include "PathFinding.h"

PathFinding::PathFinding(World world):
	m_start(0,0),
	m_goal(0,0),
	m_world(world)
{

}


PathFinding::~PathFinding(void)
{
	m_openList.clear();
	m_closedList.clear();
	m_resultPath.clear();
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
		m_resultPath.push_back(Vecteur(m_goal.x, m_goal.y));
		return;
	}
	
	//Clear
	m_openList.clear();
	m_closedList.clear();
	m_resultPath.clear();

	Node* _goalNode = new Node(m_goalNode);

	//On initialise les noeuds de départ et d'arrivée
	m_startNode = Node(m_start.x, m_start.y, m_world, NULL);
	m_goalNode = Node(m_goal.x, m_goal.y, m_world, _goalNode);

	//On commence à la node de départ
	m_currentNode = new Node(m_startNode);

	//On rajoute la node de départ à la liste des nodes disponibles
	m_closedList.push_back(m_startNode);

	//On check les nodes alentours
	this->checkNeighbourNode();
}

//Rajoute une node à la liste des nodes disponibles pour le chemin
void PathFinding::addToOpenList(float x, float y, float moveCost,  Node* parent)
{
	//On vérifie les collisions avec les obstacles fixes
	if(m_world.checkObstacle(Vecteur(x, y)))
		return;

	int id = y * m_world.worldSize + x;

	//On s'arrête là si les calculs ont déjà été fait pour cette node
	std::vector<Node>::iterator it = m_openList.begin();
	for(; it != m_openList.end(); it++)
		if(id == (it)->m_id)
			return;
	std::vector<Node>::iterator itt = m_closedList.begin();
	for(; itt != m_closedList.end(); itt++)
		if(id == (itt)->m_id)
			return;
	
	Node tempNode(x, y, m_world, parent);

	//Sinon on fait les calculs de G et H
	tempNode.G = moveCost;
	tempNode.H = tempNode.manHattanDistance(m_goalNode);
	
	m_openList.push_back(tempNode);

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
			std::vector< Node >::iterator eraseIt;
			std::vector< Node >::iterator it = m_openList.begin();
			for(; it != m_openList.end(); it++)
				if(Fvalue > (it)->getF())
				{
					Fvalue = (it)->getF();
					eraseIt = it;
				}
			m_currentNode = new Node(*eraseIt);
			//On l'enlève de la liste des nodes disponibles
			m_openList.erase(eraseIt);
			//Et on la rajoute à la liste des nodes potentiellement sur le chemin final
			m_closedList.push_back(*m_currentNode);

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
		m_goalNode.parent = m_currentNode->parent;
		Node* getPath = new Node(m_goalNode);
		for(; getPath != NULL; getPath = getPath->parent)
		{
			m_resultPath.push_back(Vecteur(getPath->m_x, getPath->m_y));
			sf::CircleShape shape( m_world.step / 4);
				shape.setFillColor(sf::Color::Blue);
				shape.setPosition(getPath->m_x, getPath->m_y);
				m_shapes.push_back(shape);
		}
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
}
