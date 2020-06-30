/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <math.h>
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
	void Inserthelper(Node *&root, int depth, std::vector<float> point, int id ){
		if(root == NULL){
			root = new Node(point, id);
		}
		else{
			
			int x_y = depth % 2;
			if( point[x_y] < root->point[x_y] ) 
				Inserthelper(root->left, depth + 1, point, id);
			else
				Inserthelper(root->right, depth + 1, point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		Inserthelper(root, 0 , point, id);

	}

	void SearchHelper(std::vector<float> target, float distanceTol, Node* root ,int depth, std::vector<int>& ids){
		if(root != NULL){
			if( (root->point[0] >= (target[0]-distanceTol)) && (root->point[0] <= (target[0]+distanceTol)) && (root->point[1] >= (target[1]-distanceTol)) && (root->point[1] <= (target[1]+distanceTol))){
				float distance = sqrt((root->point[0] - target[0]) * (root->point[0] - target[0]) + (root->point[1] - target[1]) * (root->point[1] - target[1]));
				if(distance < distanceTol)
					ids.push_back(root->id);
			}
			if( (target[depth % 2] - distanceTol) < root->point[depth % 2])
				SearchHelper(target, distanceTol , root->left, depth+1, ids);
			if( (target[depth % 2] + distanceTol) > root->point[depth%2] )
				SearchHelper(target, distanceTol , root->right, depth+1, ids);
		}
		
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		SearchHelper(target, distanceTol, root, 0, ids);
		return ids;
	}
	
	

};




