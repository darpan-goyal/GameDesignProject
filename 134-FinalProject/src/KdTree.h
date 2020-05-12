//  KdTree Template - Simple KdTree class
//
//  SJSU  - CS134 Game Development
//
//  Kevin M. Smith   04/19/20

#pragma once
#include "ofMain.h"
#include "box.h"
#include "ray.h"
#include "TreeNode.h"

//  General purpose TreeNode class
//

/*
class TreeNode {
public:
	Box box;
	vector<int> points;
	vector<TreeNode> children;    // for binary KdTree, this array has just two(2) members)
};
 */

class KdTree {
public:
	
	// Create and initilize tree given an OpenFrameWorks Mesh and # of levels.  if numLevels
	// is larger than the number of levels possible in the data, then the tree will be built to
	// its maximum depth with only one point in each leaf node.
	//
	void create(const ofMesh & mesh, int numLevels);

	//  Recursive call to subdivide mesh starting at the "node"
	//
	void subdivide(const ofMesh & mesh, TreeNode & node, int numLevels, int currentLevel);

	//  Test for intersection of Tree given a ray and root node.  If a node is hit, return true
	//  and the correspondind node in "nodeRtn"
	//
	bool intersect(const Ray &, const TreeNode & node, TreeNode & nodeRtn);

	//  Recursive calls to draw the tree
	//
	void draw(TreeNode & node, int numLevels, int level);
	void draw(int numLevels, int level) {
		draw(root, numLevels, level);
	}

	// Draw just the leaf nodes
	//  
	void drawLeafNodes(TreeNode & node);

	// utility function to draw a box in OF from a "Box" class
	//
	static void drawBox(const Box &box);

	//  Returns a bounding Box for the mesh
	//
	static Box meshBounds(const ofMesh &);

	//  Test which mesh points in "points" are inside "box"; return inside box in "pointsRtn"
	//
	int getMeshPointsInBox(const ofMesh &mesh, const vector<int> & points, Box & box, vector<int> & pointsRtn);

	//   subdvide a Box into separate boxes.  for a binary tree, subdivide into two (2) boxes.
	//
	void subDivideBox(const Box &b, vector<Box> & boxList, int level);
    
    void subDivideBoxByPoints(const vector<int> points, const Box &box, const ofMesh & mesh, vector<Box> & boxList, int level);
    
    float getMedian(vector<float> &vec);
    
    void averagePointsInLeafs() {
        cout << "totalPointsInLeaf: " << totalPointsInLeaf << endl;
         cout << "numOfLeafs: " << numOfLeafs << endl;
        cout << "Average points in leaf is: " << (totalPointsInLeaf / numOfLeafs) << endl;
        
    }

	//  local data
    //
	ofMesh mesh;
	TreeNode root;
    bool doesIntersect = false;
    bool oneFound = false;
    float totalPointsInLeaf;
    float numOfLeafs;
};
