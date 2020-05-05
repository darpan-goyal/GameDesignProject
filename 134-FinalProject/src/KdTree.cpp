//  KdTree Template - Simple KdTree class
//
//  SJSU  - CS134 Game Development
//
//  Kevin M. Smith   04/19/20

//  **Important:  Vertices (x, y, z) in the mesh are stored in the Tree node as an integer index.
//  to read the 3D vertex point from the mesh given index i,  use the function ofMesh::getVertex(i);  See
//  KdTree::meshBounds() for an example of usage;
//
//

#include "KdTree.h"
#include <algorithm>
 
// draw KdTree (recursively)
//
void KdTree::draw(TreeNode & node, int numLevels, int level) {
	if (level >= numLevels) return;
	drawBox(node.box);
	level++;
	for (int i = 0; i < node.children.size(); i++) {
		draw(node.children[i], numLevels, level);
	}
}

// draw only leaf Nodes
//
void KdTree::drawLeafNodes(TreeNode & node) {
    if(node.children.size() <= 0) {
        drawBox(node.box);
        numOfLeafs++;
        totalPointsInLeaf += node.points.size();
    }
     
    for (int i = 0; i < node.children.size(); i++) {
        drawLeafNodes(node.children[i]);
    }
}


//draw a box from a "Box" class  
//
void KdTree::drawBox(const Box &box) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	ofVec3f p = ofVec3f(center.x(), center.y(), center.z());
	float w = size.x();
	float h = size.y();
	float d = size.z();
	ofDrawBox(p, w, h, d);
}

// return a Mesh Bounding Box for the entire Mesh
//
Box KdTree::meshBounds(const ofMesh & mesh) {
	int n = mesh.getNumVertices();
	ofVec3f v = mesh.getVertex(0);
	ofVec3f max = v;
	ofVec3f min = v;
	for (int i = 1; i < n; i++) {
		ofVec3f v = mesh.getVertex(i);

		if (v.x > max.x) max.x = v.x;
		else if (v.x < min.x) min.x = v.x;

		if (v.y > max.y) max.y = v.y;
		else if (v.y < min.y) min.y = v.y;

		if (v.z > max.z) max.z = v.z;
		else if (v.z < min.z) min.z = v.z;
	}
	cout << "vertices: " << n << endl;
	return Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
}

// getMeshPointsInBox:  return an array of indices to points in mesh that are contained 
//                      inside the Box.  Return count of points found;
//
int KdTree::getMeshPointsInBox(const ofMesh & mesh, const vector<int>& points, Box & box, vector<int> & pointsRtn)
{
    int numPointsInside = 0;
    for(int i = 0; i < points.size(); i++) {
        ofVec3f pointToCheck = mesh.getVertex(points[i]);
        if(box.inside(Vector3(pointToCheck.x, pointToCheck.y, pointToCheck.z))) {
            pointsRtn.push_back(points[i]);
            numPointsInside++;
        }
    }
	return numPointsInside;
}

//  Subdivide a Box; return children in  boxList
//
void KdTree::subDivideBox(const Box &box, vector<Box> & boxList, int level) {
    Vector3 boxMin = box.parameters[0];
    Vector3 boxMax = box.parameters[1];
    Vector3 boxSize = boxMax - boxMin;
    Vector3 center = boxSize / 2 + boxMin;
        
    Box childOne;
    Box childTwo;
    
    if(level % 3 == 1) {
        childOne = Box(boxMin, Vector3(center.x(), boxMax.y(), boxMax.z()));
        childTwo = Box(Vector3(center.x(), boxMin.y(), boxMin.z()), boxMax);
    }
    else if(level % 3 == 2) {
        childOne = Box(boxMin, Vector3(boxMax.x(), boxMax.y(), center.z()));
        childTwo = Box(Vector3(boxMin.x(), boxMin.y(), center.z()), boxMax);
    }
    else {
        childOne = Box(boxMin, Vector3(boxMax.x(), center.y(), boxMax.z()));
        childTwo = Box(Vector3(boxMin.x(), center.y(), boxMin.z()), boxMax);
    }
        
    boxList.push_back(childOne);
    boxList.push_back(childTwo);
}

void KdTree::subDivideBoxByPoints(const vector<int> points, const Box &box, const ofMesh & mesh, vector<Box> & boxList, int level) {
    
    Vector3 boxMin = box.parameters[0];
    Vector3 boxMax = box.parameters[1];
    Vector3 boxSize = boxMax - boxMin;
    Vector3 center = boxSize / 2 + boxMin;
        
    Box childOne;
    Box childTwo;
    
    vector<float> xyzVals;
    
    if(level % 3 == 1) {
        for(int i = 0; i < points.size(); i++)
            xyzVals.push_back((mesh.getVertex(points[i])).x);
        
        float middleXVal = getMedian(xyzVals);
        childOne = Box(boxMin, Vector3(middleXVal, boxMax.y(), boxMax.z()));
        childTwo = Box(Vector3(middleXVal, boxMin.y(), boxMin.z()), boxMax);
    }
    else if(level % 3 == 2) {
        for(int i = 0; i < points.size(); i++)
            xyzVals.push_back((mesh.getVertex(points[i])).z);
        
        float middleZVal = getMedian(xyzVals);
        childOne = Box(boxMin, Vector3(boxMax.x(), boxMax.y(), middleZVal));
        childTwo = Box(Vector3(boxMin.x(), boxMin.y(), middleZVal), boxMax);
    }
    else {
        for(int i = 0; i < points.size(); i++)
            xyzVals.push_back((mesh.getVertex(points[i])).y);
        float middleYVal = getMedian(xyzVals);
        childOne = Box(boxMin, Vector3(boxMax.x(), middleYVal, boxMax.z()));
        childTwo = Box(Vector3(boxMin.x(), middleYVal, boxMin.z()), boxMax);
    }
    
    boxList.push_back(childOne);
    boxList.push_back(childTwo);
}

float KdTree::getMedian(vector<float> &vec) {
    int middleOfVec = vec.size() / 2;
    nth_element(vec.begin(), vec.begin() + middleOfVec, vec.end());
    
    if(vec.size() % 2 == 0) {
        nth_element(vec.begin(), vec.begin() + middleOfVec - 1, vec.end());
        float evenMedian = (vec[middleOfVec] + vec[middleOfVec - 1]) / 2.0;
        return evenMedian;
    }
    else
        return vec[middleOfVec];
}


void KdTree::create(const ofMesh & geo, int numLevels) {
	mesh = geo;
	int level = 0;

	root.box = meshBounds(mesh);
	for (int i = 0; i < mesh.getNumVertices(); i++) {
		root.points.push_back(i);
	}
	level++;

	subdivide(mesh, root, numLevels, level);
}

void KdTree::subdivide(const ofMesh & mesh, TreeNode & node, int numLevels, int level) {
    if(level >= numLevels) return;
    
    if(node.points.size() == 1) return;
    
    vector<Box> childBoxes;
    subDivideBox(node.box, childBoxes, level);
    //subDivideBoxByPoints(node.points, node.box, mesh, childBoxes, level);
    
    TreeNode childOne;
    TreeNode childTwo;
    
    childOne.box = childBoxes[0];
    childTwo.box = childBoxes[1];
    
    int numPointsB1 = getMeshPointsInBox(mesh, node.points, childOne.box, childOne.points);
    int numPointsB2 = getMeshPointsInBox(mesh, node.points, childTwo.box, childTwo.points);
    
    if(numPointsB1 > 0)
        node.children.push_back(childOne);
    if(numPointsB2 > 0)
        node.children.push_back(childTwo);
        
    if(node.children.size() > 0) {
        for(int i = 0; i < node.children.size(); i++) {
            if(node.children[i].points.size() > 1)
                subdivide(mesh, node.children[i], numLevels, level + 1);
        }
    }
    else return;
}

bool KdTree::intersect(const Ray &ray, const TreeNode & node, TreeNode & nodeRtn) {
    if(node.box.intersect(ray, 0, 1000))
    {
       for(TreeNode child: node.children) {
           if(child.points.size() > 1)
           {
               if(intersect(ray, child, nodeRtn))
                   return true;
            }
           else if(child.points.size() == 1)
           {
               nodeRtn = child;
               return true;
           }
        }
        return false;
    }
}
