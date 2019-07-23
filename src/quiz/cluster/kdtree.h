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

	void insertHelper(Node ** currNode, uint depth,std::vector<float> point, int id)
	{
		if(*currNode == NULL)
		{
			*currNode = new Node(point, id);
		}
		else
		{
			if(point[depth % 2] >= (*currNode)->point[depth % 2])
			{
				insertHelper(&((*currNode)->right), depth + 1, point, id);
			}
			else{
				insertHelper(&((*currNode)->left), depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		//insertHelper(&root, 0, point, id);
		Node * newNode = new Node(point, id);
		int depth = 0;
		if(root == NULL)
		{
			root = newNode;
		}
		else
		{
			Node * curr = root;
			bool isNodeAssigned = false;
			while(isNodeAssigned == false)
			{
				if(point[depth % 2] >= curr->point[depth % 2]){
					if(curr->right == NULL)
					{
						curr->right = newNode;
						isNodeAssigned = true;
					}
					else
						curr = curr->right;
					
				}
				else
				{
					if(curr->left == NULL)
					{
						curr->left = newNode;
						isNodeAssigned = true;
					}
					else
						curr = curr->left;
				}
				depth++;
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




