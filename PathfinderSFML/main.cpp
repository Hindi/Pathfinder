#include "stdAfx.h"

#include <SFML/Graphics.hpp>

#include "PathFinding.h"
#include "Vecteur.hpp"
#include "World.hpp"

int _tmain(int argc, _TCHAR* argv[])
{
	//On lance la recherche de chemin
	sf::RenderWindow window(sf::VideoMode(1024 , 768), "Pathfinding A* - Cours siteduzero.com");
	//Espace de jeu
	World world;
	//On instancie le pathFinder
	PathFinding pathfinder(world);
	//On lance la recherche de chemin
	pathfinder.findPath(Vecteur(world.step,world.step), Vecteur(11*world.step,2*world.step));
	//On créé une liste Vecteur et on récupère le chemin du pathfinder
	std::vector<Vecteur*> path(pathfinder.getPath());
	
	//On affiche les coordonées des points du chemin
	std::vector<Vecteur*>::reverse_iterator it = path.rbegin();
	/*for(; it != path.rend(); it++)
		std::cout << (*it)->x << " " << (*it)->y << std::endl;*/
	//On créé les obstacles
	sf::RectangleShape rect1(sf::Vector2f(50, 300));
	rect1.setFillColor(sf::Color::Black);
	rect1.setPosition(200, 0);
	sf::RectangleShape rect2(sf::Vector2f(50, 200));
	rect2.setFillColor(sf::Color::Black);
	rect2.setPosition(350, 200);
	
	//Boucle d'affichage
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