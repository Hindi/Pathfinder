#pragma once

class Vecteur
{
	public:
		Vecteur(void) {}
		~Vecteur(void) {}
		
		Vecteur(int nX, int nY)
			{
				x = nX;
				y =nY;
			}
		Vecteur(const Vecteur &vector)
			{
				x = vector.x;
				y = vector.y;
			}

		Vecteur& operator=(const Vecteur &vector)
			{
				x = vector.x;
				y = vector.y;
				return *this;
			}

		bool operator==(const Vecteur &vector)
			{
				return (x == vector.x && y == vector.y);
			}
		
		int x;
		int y;

};

