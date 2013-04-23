#include "stdAfx.h"
#include "PathFinding.h"
#include "Vecteur.hpp"
#include "World.hpp"
#include <iostream>

int _tmain(int argc, _TCHAR* argv[])
{
	World world;
	//On instancie le pathFinder
	PathFinding pathfinder(world);
	//On lance la recherche de chemin
	pathfinder.findPath(Vecteur(10,10), Vecteur(500,300));
	//On créé une liste Vecteur et on récupère le chemin du pathfinder
	std::vector<Vecteur> path(pathfinder.getPath());

	//On affiche les coordonées des points du chemin
	std::vector<Vecteur>::reverse_iterator it = path.rbegin();
	for(; it != path.rend(); it++)
		std::cout << (it)->x << " " << (it)->y << std::endl;
	
	
	while(42)
	{}
	return 0;
}