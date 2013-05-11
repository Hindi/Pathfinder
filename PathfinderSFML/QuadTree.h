#pragma once
#include "Vecteur.hpp"
#include <unordered_set>

struct BoundingBox{
	BoundingBox();
	BoundingBox(Vecteur position, Vecteur size, int idSize);
	bool checkBoundingBox(int id);
	bool checkBoundingBox(Vecteur position);

	Vecteur m_position;
	Vecteur m_size;
	int m_idSize;
	std::unordered_set<int> m_obstaclesId;
};

class QuadTree
{
	public:
		QuadTree();
		QuadTree(Vecteur worldSize, int idSize);
		~QuadTree(void);
		void addObstacleId(int id);
		bool containsObstacleId(int id);

	private:
		std::vector<BoundingBox> m_boundingBox;
		Vecteur m_worldSize;
};

