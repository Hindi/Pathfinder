#pragma once

#include "Vecteur.hpp"
#include "QuadTree.h"


class World
{
	public:
		World(void):
			step(20),
			worldWidth(200),
			worldLength(300),
			worldSize(1000)
		{
			quadTree = QuadTree(Vecteur(worldWidth, worldLength), worldSize);
			loadMap();
		}
		
		~World(void) {}

		void loadMap()
		{
			for(int i(0); i <= worldLength; i += step)
				for(int j(0); j <= worldWidth; j +=  step)
					if(checkObstacle(Vecteur(i,j)))
						quadTree.addObstacleId(j*worldSize + i);
		}

		bool checkObstacle(Vecteur pos)
		{
			//Bordure de map :
			if(pos.x >= worldLength || pos.x <= 0 || pos.y >= worldWidth || pos.y <= 0)
				return true;

			//Deux obstacles
			if(pos.x >= 4*step && pos.x <= 4*step && pos.y >= 0*step && pos.y <= 5*step)
				return true;
	
			if(pos.x >= 8*step && pos.x <= 8*step && pos.y >= 3*step && pos.y <= 6*step)
				return true;

			return false;
		}

		bool checkObstacleId(int id)
		{
			return quadTree.containsObstacleId(id);
		}

		const int step;
		const int worldSize;
		const int worldWidth;
		const int worldLength;
		QuadTree quadTree;
};

