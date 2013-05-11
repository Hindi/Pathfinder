#pragma once

#include "Vecteur.hpp"

class World
{
	public:
		World(void):
			step(20),
			worldWidth(200),
			worldLength(300),
			worldSize(1000)
		{
		}
		
		~World(void) {}

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

		const int step;
		const int worldSize;
		const int worldWidth;
		const int worldLength;
};