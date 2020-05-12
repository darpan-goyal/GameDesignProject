#pragma once
#include "ofMain.h"
#include "box.h"
#include "ray.h"

class TreeNode {
public:
	Box box;
	vector<int> points;
	vector<TreeNode> children;    // for binary KdTree, this array has just two(2) members)
};
