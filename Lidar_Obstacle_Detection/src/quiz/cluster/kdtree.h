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

	void insertHelper(Node** node, int nid, std::vector<float> point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node(point,id);
		}
		else
		{
			//Check
			if(point[nid%2] < (*node)->point[nid%2])
			{
				insertHelper(&((*node)->left),nid+1,point, id);
			}
			else
			{
				insertHelper(&((*node)->right),nid+1,point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, 0, point, id);
		
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

	}

	void insertCheckHelper(Node** node, std::vector<std::pair<std::vector<float>, int>>::iterator start, std::vector<std::pair<std::vector<float>, int>>::iterator end, int currentLevel)
	{
		if(start>=end)
			return ;
		
		int axis = currentLevel%2;

		auto cmp =[axis](const std::pair<std::vector<float>,int>& p1, const std::pair<std::vector<float>,int>& p2) {
			return p1.first[axis]< p2.first[axis];
		};

		auto len = end - start;
		auto mid = start + (len/2);
		std::nth_element(start, mid, end, cmp);

		//while(mid > start && (*(mid-1))[axis] == (*mid)[axis])
		while(mid > start && (mid-1)->first[axis] == mid->first[axis])
		{
			--mid;
		}

		*node = new Node(mid->first,mid->second);
		std::cout<<"{ "<<mid->first[0]<<", "<<mid->first[1]<<"}"<<std::endl;
		std::cout<<"Left"<<std::endl;
		insertCheckHelper(&((*node)->left),start, mid, currentLevel+1);
		std::cout<<"Right"<<std::endl;
		insertCheckHelper(&((*node)->right), mid+1, end, currentLevel+1);
		

	}
    
	void insertCheck(std::vector<std::pair<std::vector<float>, int>>::iterator start, std::vector<std::pair<std::vector<float>, int>>::iterator end, int currentLevel)
	{
		insertCheckHelper(&root, start, end, currentLevel);
	}

	void searchHelper(Node* node, int nid, std::vector<float>& target, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			/*
			std::cout<<"{"<<node->point[0]<<", "<<node->point[1]<<"},   {"<<target[0]<<","<<target[1]<<"}"<<endl;
			std::cout<<(node->point[0]>= (target[0] - distanceTol));
			std::cout<<(node->point[0]<= (target[0] + distanceTol));
			std::cout<<(node->point[1]>= (target[1] - distanceTol));
			std::cout<<(node->point[1]<= (target[1] + distanceTol))<<endl;
			*/
			if( (node->point[0]>= (target[0] - distanceTol) )&& (node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1] - distanceTol)) && (node->point[1] <= (target[1] + distanceTol)) )
			{
				float diff_x = node->point[0] - target[0];
				float diff_y = node->point[1] - target[1];
				float diff = sqrt((diff_x*diff_x) + (diff_y*diff_y));
				//std::cout<<"Diff:"<<diff<<endl;
				if (diff <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			if((target[nid%2] - distanceTol) < node->point[nid%2])
			{
				//std::cout<<"Left"<<endl;
				searchHelper(node->left, nid+1, target, distanceTol,ids);
			}
			if((target[nid%2] + distanceTol) > node->point[nid%2])
			{
				//std::cout<<"Right"<<endl;
				searchHelper(node->right, nid+1, target, distanceTol,ids);
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




