#pragma once

#include "ofMain.h"
#include  "ofxAssimpModelLoader.h"
#include "box.h"
#include "ray.h"
#include "KdTree.h"
#include "ofxGui.h"
#include "ParticleSystem.h"
#include "ParticleEmitter.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void drawAxis(ofVec3f);
		void initLightingAndMaterials();
		void savePicture();
		void toggleWireframeMode();
		void togglePointsDisplay();
		void toggleSelectTerrain();
		void setCameraTarget();
		glm::vec3 getMousePointOnPlane();
		void drawBox(const Box &box);
		Box meshBounds(const ofMesh &);
		

		ofEasyCam cam;
		ofCamera top;
		ofCamera *theCam;
		ofxAssimpModelLoader terrain, lander;
		ofLight light;
		Box boundingBox;
		Box landerBounds;
	
		bool bAltKeyDown;
		bool bCtrlKeyDown;
		bool bWireframe;
		bool bDisplayPoints;
		bool bPointSelected;
		
		bool bLanderLoaded = false;
		bool bTerrainSelected;
		bool bLanderSelected = false;
        bool bInDrag = false;
        glm::vec3 getMousePointOnPlane(glm::vec3 planePt, glm::vec3 planeNorm);
	
		ofVec3f selectedPoint;
		ofVec3f intersectPoint;
		
		glm::vec3 mouseDownPos;
        glm::vec3 mouseLastPos;


		const float selectionRange = 4.0;

		KdTree kdtree;
    
        ofxPanel gui;
        ofxFloatSlider drawLevel;
        bool printOnce = true;
        TreeNode nodeSelected;
        Box selectedBox;
    
        // Midterm Code
        Particle *lunarModel;
        ParticleSystem *lunarModelSys;
        ParticleEmitter *thrustEmitter;
        ThrusterForce *tForce;
        float lmAngle;
        bool rotateCW = false;
        bool rotateCCW = false;
        glm::vec3 headingVector;
};
