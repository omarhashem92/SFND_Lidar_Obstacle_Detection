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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




