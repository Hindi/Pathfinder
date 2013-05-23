#include "PathFinding.h"
#include "Vecteur.hpp"
#include "World.hpp"
#include <SFML/Graphics.hpp>

int main()
{
	sf::RenderWindow window(sf::VideoMode(1024 , 768), "Pathfinding A* - Cours siteduzero.com");
	//Espace de jeu
	World world;
	//On instancie le pathFinder
	PathFinding pathfinder(world);
	pathfinder.addEnemyPosition(60, 110, 20);
	//On créé une liste Vecteur et on récupère le chemin du pathfinder
	std::vector<Vecteur> path;
	
	//On lance la recherche de chemin
	path = pathfinder.findPath(Vecteur(10,40), Vecteur(250,50));
	
	//On affiche les coordonées des points du chemin
	/*std::vector<Vecteur>::iterator it = path.begin();
	for(; it != path.end(); it++)
		std::cout << (*it).x << " " << (*it).y << std::endl;*/
	
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