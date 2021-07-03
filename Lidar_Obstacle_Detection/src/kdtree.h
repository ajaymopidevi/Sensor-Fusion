
//#include "../../render/render.h"


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

	void buildTreeHelper(Node** node, std::vector<std::pair<std::vector<float>, int>>::iterator start, std::vector<std::pair<std::vector<float>, int>>::iterator end, int currentLevel)
	{
		if(start>=end)
			return ;
		
		int axis = currentLevel%3;

		auto cmp =[axis](const std::pair<std::vector<float>,int>& p1, const std::pair<std::vector<float>,int>& p2) {
			return p1.first[axis]< p2.first[axis];
		};

		auto len = end - start;
		auto mid = start + (len/2);
		std::nth_element(start, mid, end, cmp);

		while(mid > start && (mid-1)->first[axis] == mid->first[axis])
		{
			--mid;
		}

		*node = new Node(mid->first,mid->second);
		buildTreeHelper(&((*node)->left),start, mid, currentLevel+1);
		buildTreeHelper(&((*node)->right), mid+1, end, currentLevel+1);
		

	}
    
	void buildTree(std::vector<std::pair<std::vector<float>, int>>::iterator start, std::vector<std::pair<std::vector<float>, int>>::iterator end)
	{
		//Build a balanced KD-Tree
		//Balanced KD-Tree reduces effective time for each search
		buildTreeHelper(&root, start, end, 0);
	}

	void insertHelper(Node** node, int nid, std::vector<float> point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node(point,id);
		}
		else
		{
			
			uint c = nid%3;
			if(point[c] < (*node)->point[c])
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
		//Insert a point in KD-Tree
		insertHelper(&root, 0, point, id);
		
	}
    
	void searchHelper(Node* node, int nid, std::vector<float> target, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
	
			if( (node->point[0]>= (target[0] - distanceTol) )&& 
			    (node->point[0] <= (target[0] + distanceTol)) && 
				(node->point[1] >= (target[1] - distanceTol)) && 
				(node->point[1] <= (target[1] + distanceTol)) &&
				(node->point[2] >= (target[2] - distanceTol)) && 
				(node->point[2] <= (target[2] + distanceTol)) )
			{
				float diff_x = node->point[0] - target[0];
				float diff_y = node->point[1] - target[1];
				float diff_z = node->point[2] - target[2];
				float diff = sqrt((diff_x*diff_x) + (diff_y*diff_y) );
				if (diff <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}
			uint c = nid%3;

			if((target[c] - distanceTol) < node->point[c])
			{
				searchHelper(node->left, nid+1, target, distanceTol,ids);
			}
			if((target[c] + distanceTol) > node->point[c])
			{
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




