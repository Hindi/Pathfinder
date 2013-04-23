#include "stdAfx.h"
#include "PathFinding.h"
#include "Vecteur.hpp"
#include "World.hpp"
#include <iostream>

int _tmain(int argc, _TCHAR* argv[])
{
	sf::RenderWindow window(sf::VideoMode(1024 , 768), "Pathfinding A*");
	World world;
	//On instancie le pathFinder
	PathFinding pathfinder(world);
	//On lance la recherche de chemin
	pathfinder.findPath(Vecteur(10,10), Vecteur(500,100));
	//On créé une liste Vecteur et on récupère le chemin du pathfinder
	std::vector<Vecteur> path(pathfinder.getPath());

	//On affiche les coordonées des points du chemin
	std::vector<Vecteur>::reverse_iterator it = path.rbegin();
	for(; it != path.rend(); it++)
		std::cout << (it)->x << " " << (it)->y << std::endl;
	
	while (window.isOpen())
    {
		//Enlever l'affichage la frame précédent
		window.clear(sf::Color::White);
		sf::Event event;
		//Gesion de l'évènement "fermer la fenêtre"
		while (window.pollEvent(event))
			if (event.type == sf::Event::Closed)
                window.close();
		//On dessine nos cercles
		pathfinder.draw(window);
		//On dessine les obstacles
		//window.draw(rect1);
		//window.draw(rect2);
		//Afficher nos objets
        window.display();
	}
	return 0;
}