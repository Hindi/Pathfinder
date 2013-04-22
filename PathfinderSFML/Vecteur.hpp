#pragma once

class Vecteur
{
	public:
		Vecteur(void) {}
		~Vecteur(void) {}
		
		Vecteur(float nX, float nY)
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
		
		float x;
		float y;

};

