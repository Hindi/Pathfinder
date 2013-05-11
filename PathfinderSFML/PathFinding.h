#include "stdafx.h"

#include <vector>
#include <SFML/Graphics.hpp>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include <sstream>
#include <string>

#include "Vecteur.hpp"
#include "Node.h"

/*
	Permet de trouver un chemin entre les positions 
	de d�part et d'arriv�e en prenant en compte les
	obstacles fixes.

	M�thode :
	- Utiliser setStartAndGoal() pour d�finir les 
		positions de d�part et d'arriv�e. 
	- Utiliser findPath() (thread� de pr�f�rence)
	- Utiliser path() pour r�cup�rer le chemin trouv�

*/

class PathFinding
{
	public:
		PathFinding(World world);
		~PathFinding(void);
		
		//Sert a lancer la recherche, fait les initialisations
		void findPath(Vecteur start, Vecteur goal);

		//Renvoie le chemin vers l'objectif
		std::vector<Vecteur> getPath();

		//Dessine nos objets � l'�cran
		void draw(sf::RenderWindow &window);

	private:
		//Rajoute une node � la liste des noeds disponibles pour le chemin
		void addToOpenList(float x, float y, float moveCost, Node* parent);

		//Lance les calculs sur les nodes alentours, v�rifie si l'on est arriv�
		void checkNeighbourNode();

		bool lineOfSight(std::shared_ptr<Node> startNode, std::shared_ptr<Node> goalNode);

		//Permet de trouver une node de d�part convenable
		void findStartNode();

		//Nodes de d�part et d'arriv�e
		Node m_startNode;
		Node m_goalNode;

		//Node sur laquelle s'effectue le calcul
		Node* m_currentNode;

		//Listes des nodes disponibles pour les calculs e pour le chemin final
		std::unordered_set<int> m_openList;
		std::unordered_set<int> m_closedList;

		//Chemin final
		std::vector<Vecteur> m_resultPath;

		//Sert aux d�bug pour stocker les cercles affich�s lors du calcul
		std::vector<sf::CircleShape> m_shapes;
		std::vector<sf::Text> m_text;
		std::unordered_map<int, Node> m_grille;

		World m_world;
};