/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node **currNode, uint depth, std::vector<float> point, int id) 
	{
			if(*currNode == NULL)
		{
			*currNode = new Node(point, id);
		}
		else
		{
			uint cd = depth % 2;
			if(point[cd] < ((*currNode)->point[cd]))
			{
				insertHelper(&((*currNode)->left), depth+1, point, id);
			}
			else
			{
				insertHelper(&((*currNode)->right), depth+1, point, id);
			}
						
			
		}
	}

bool isInsideBox(std::vector<float> point, std::vector<float> target, float distanceTol)
	{
		for(int dimension = 0;dimension < point.size();++dimension)
		{
			float minN = target[dimension] - distanceTol;
			float maxN = target[dimension] + distanceTol;
			if(point[dimension] > maxN || point[dimension] < minN)
			{
				return false;
			}
		}
		return true;
	}

void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
		
	}

float computeDist(std::vector<float> point1, std::vector<float> point2)
	{
		float retDist = 0;
		for(int dimension = 0;dimension < point1.size();++dimension)
		{
			float subN = point1[dimension] - point2[dimension];
			retDist += subN * subN;
		}

		retDist = sqrt(retDist);
		return retDist;
	}


void searchHelper(Node *currNode, uint level, std::vector<float> target, float distanceTol, std::vector<int> &ids)
{
	if(currNode != NULL)
	{
		if(isInsideBox(currNode->point, target, distanceTol))
		{
			float dist = computeDist(target, currNode->point);
			if(dist <= distanceTol)
			{
				ids.push_back(currNode->id);
			}
		}

		uint numDimensions = target.size();
		if(currNode->point[level%numDimensions] >= (target[level%numDimensions]-distanceTol))
		{
			searchHelper(currNode->left, level + 1, target, distanceTol, ids);
		}
		if(currNode->point[level%numDimensions] <= (target[level%numDimensions]+distanceTol))
		{
			searchHelper(currNode->right, level + 1, target, distanceTol, ids);
		}

	}

}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);
		return ids;
	}
	

};




