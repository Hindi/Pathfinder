#pragma once

#include "Vecteur.hpp"

class World
{
	public:
		World(void):
			step(20),
			worldSize(2000)
		{
		}
			~World(void) {}

		bool checkObstacle(Vecteur pos)
		{
			//Bordure de map :
			if(pos.x >= worldSize || pos.x <= 0 || pos.y >= worldSize || pos.y <= 0)
				return true;

			//Deux obstacles
			if(pos.x >= 200 && pos.x <= 250 && pos.y >= 0 && pos.y <= 300)
				return true;
	
			if(pos.x >= 350 && pos.x <= 400 && pos.y >= 200 && pos.y <= 400)
				return true;

			return false;
		}

		const int step;
		const int worldSize;
};

