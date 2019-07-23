/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;//0: x, 1: y, 2:z
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

	bool isInsideBox(std::vector<float> point, std::vector<float> target, float distanceTol)
	{
		bool isInside = false;
		float minX = target[0] - distanceTol;
		float maxX = target[0] + distanceTol;
		float minY = target[1] - distanceTol;
		float maxY = target[1] + distanceTol;
		if((point[0] <= maxX && point[0] >= minX) && (point[1] <= maxY && point[1] >= minY))
		{
			isInside = true;
		}
		return isInside;
	}

	float computeDist(std::vector<float> point1, std::vector<float> point2)
	{
		float subX = point1[0] - point2[0];
		float subY = point1[1] - point2[1];
		float retDist = sqrt(subX * subX + subY * subY);
		return retDist;
	}

	void searchRec(Node* currNode, uint level, std::vector<float> target, float distanceTol, std::vector<int>& ids)
	{
		if(currNode == NULL)
			return;
		if(isInsideBox(currNode->point, target, distanceTol))
		{
			float dist = computeDist(target, currNode->point);
			if(dist <= distanceTol)
			{
				ids.push_back(currNode->id);
			}
		}
		if(currNode->point[level%2] >= (target[level%2]-distanceTol))
		{
			searchRec(currNode->left, level + 1, target, distanceTol, ids);
		}
		if(currNode->point[level%2] <= (target[level%2]+distanceTol))
		{
			searchRec(currNode->right, level + 1, target, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		Node* currNode = root;
		searchRec(root, 0, target, distanceTol, ids);
		
		return ids;
	}
	

};




