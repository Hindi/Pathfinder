#pragma once

#include "Vecteur.hpp"

class World
{
	public:
		World(void):
			step(10),
			worldWidth(200),
			length(300),
			worldSize(1000)
		{
		}
		
		~World(void) {}

		bool checkObstacle(Vecteur pos)
		{
			//Bordure de map :
			if(pos.x >= length || pos.x <= 0 || pos.y >= worldWidth || pos.y <= 0)
				return true;

			//Deux obstacles
			if(pos.x >= 6*step && pos.x <= 7*step && pos.y >= 0*step && pos.y <= 10*step)
				return true;
	
			if(pos.x >= 10*step && pos.x <= 12*step && pos.y >= 6*step && pos.y <= 13*step)
				return true;

			return false;
		}

		const int step;
		const int worldSize;
		const int worldWidth;
		const int length;
};