#include <vector>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include <sstream>
#include <string.h>

#include "Vecteur.hpp"
#include "Node.h"
#include <math.h>
#include <SFML/Graphics.hpp>

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
		std::vector<Vecteur> findPath(Vecteur start, Vecteur goal);

		//Renvoie le chemin vers l'objectif
		std::vector<Vecteur> getPath();
		
		//Retire les nodes occup�es par les ennemis
		void addEnemyPosition(int x, int y, int rayon);	
		
		void draw(sf::RenderWindow &window);
	private:
		//Rajoute une node � la liste des noeds disponibles pour le chemin
		void addToOpenList(float x, float y, float moveCost, Node* parent);

		//Lance les calculs sur les nodes alentours, v�rifie si l'on est arriv�
		void checkNeighbourNode();

		bool lineOfSight(Vecteur start, Vecteur goal);
		
		void smoothPath();
		void testVisibility();

		//Permet de trouver une node de d�part convenable
		Vecteur findCloseNode(Vecteur pos);
		Vecteur findCloseNode(int x, int y);
		
		//Inverse l'ordre des coordonn�es dans le chemin obtenu
		void revertPath();
		
		void changeCoordinateSystem();


		//Nodes de d�part et d'arriv�e
		Node m_startNode;
		Node m_goalNode;

		//Node sur laquelle s'effectue le calcul
		Node* m_currentNode;

		//Listes des nodes disponibles pour les calculs e pour le chemin final
		std::unordered_set<int> m_openList;
		std::unordered_set<int> m_closedList;
		std::unordered_set<int> m_enemyList;

		//Chemin final
		std::vector<Vecteur> m_resultPath;

		std::unordered_map<int, Node> m_grille;
		std::vector<sf::CircleShape> m_shapes;
		std::unordered_set<int> m_staticObstacles;

		World m_world;
};