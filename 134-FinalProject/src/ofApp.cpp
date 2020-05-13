//  Student Name:   Sivam Agarwalla, Darpran Goyal
//  Date: <date of last version>


#include "ofApp.h"
#include "Util.h"
#include <glm/gtx/intersect.hpp>


//--------------------------------------------------------------
// setup scene, lighting, state and load geometry
//
void ofApp::setup(){

	bWireframe = false;
	bDisplayPoints = false;
	bAltKeyDown = false;
	bCtrlKeyDown = false;
	bLanderLoaded = false;
	bTerrainSelected = true;
    
//	ofSetWindowShape(1024, 768);
    
	cam.setDistance(10);
	cam.setNearClip(.1);
	cam.setFov(65.5);   // approx equivalent to 28mm in 35mm format
	cam.disableMouseInput();
    
    ofSetVerticalSync(true);
	ofEnableSmoothing();
	ofEnableDepthTest();
    
    // Loading a background image
    bBackgroundLoaded = backgroundImage.load("images/starfield-purple1.jpg");

	top.setPosition(0, 25, 0);
	top.lookAt(glm::vec3(0, 0, 0));
	top.setNearClip(.1);
	top.setFov(65.5);   // approx equivalent to 28mm in 35mm format

	theCam = &cam;

	// setup rudimentary lighting 
	//
	initLightingAndMaterials();

    terrain.loadModel("geo/moon-houdini.obj");
	terrain.setScaleNormalization(false);

	boundingBox = meshBounds(terrain.getMesh(0));

	// create KdTree for terrain
	//
    float timeBefore = ofGetElapsedTimef();
	//kdtree.create(terrain.getMesh(0), 40);
    octree.create(terrain.getMesh(0), 15);
    float timeAfter = ofGetElapsedTimef();
    cout << "Time taken to build tree in MS: " << (timeAfter - timeBefore) << endl;
    
    gui.setup();
    gui.add(drawLevel.setup("Draw Level", 1, 1, 15));
    
    // Midterm Code
    /*
    if (lander.loadModel("geo/lander.obj")) {
        lander.setScaleNormalization(false);
        //lander.setScale(.5, .5, .5);
        //lander.setRotation(0, -180, 1, 0, 0);
        lander.setPosition(0, 0, 0);

        bLanderLoaded = true;
    }
    else {
        cout << "Error: Can't load model" << "geo/lander.obj" << endl;
        ofExit(0);
    }
     */
    
    lmAngle = 0;
    headingVector = glm::vec3(0, 0, 0);
    
    // setup LEM
    //
    lunarModel = new Particle();
    lunarModelSys = new ParticleSystem();
    
    lunarModel->position = glm::vec3(0, 0, 0);
    lunarModel->radius = 0.25;
    lunarModel->lifespan = -1;
    lunarModelSys->add(*lunarModel);
    tForce = new ThrusterForce(glm::vec3(0, 0, 0));
    tForce->applied = true;
    lunarModelSys->addForce(tForce);
    lunarModelSys->addForce(new TurbulenceForce(glm::vec3(-0.3, -0.3, -0.3), glm::vec3(0.3, 0.3, 0.3)));
    
    thrustEmitter = new ParticleEmitter();
    thrustEmitter->type = DiscEmitter;
    thrustEmitter->velocity = glm::vec3(0, -3, 0);
    thrustEmitter->rate = 20;
    thrustEmitter->randomLife = true;
    thrustEmitter->lifeMinMax = ofVec3f(0.15, 0.45);
    thrustEmitter->radius = 0.18;
    thrustEmitter->particleRadius = 0.01;
    thrustEmitter->groupSize = 50;
}

//--------------------------------------------------------------
// incrementally update scene (animation)
//
void ofApp::update() {
    // Midterm Code
    lunarModelSys->update();
    lander.setPosition(lunarModelSys->particles[0].position.x, lunarModelSys->particles[0].position.y, lunarModelSys->particles[0].position.z);
    
    thrustEmitter->update();
    thrustEmitter->setPosition(glm::vec3(lunarModelSys->particles[0].position.x, lunarModelSys->particles[0].position.y, lunarModelSys->particles[0].position.z));
    
    // Altitude Detection
    if(bLanderLoaded) {
        
        if(ofGetElapsedTimeMillis() - intersectTime > 500) {
            intersectTime = ofGetElapsedTimeMillis();
            Vector3 origin = Vector3(lander.getPosition().x, lander.getPosition().y, lander.getPosition().z);
            Vector3 direction = Vector3(0, -1, 0);
            Ray altDetect = Ray(origin, direction);
            TreeNode localNode;
        
            //if(kdtree.intersect(altDetect, kdtree.root, localNode)) {
            if(octree.intersect(altDetect, octree.root, localNode)) {
                landerAlt = lunarModelSys->particles[0].position.y - localNode.box.center().y();
            }
        }
    
        // Check for surface collision
        // Get bottom points for the landerBounds box.
        
        ofVec3f velocity = lunarModelSys->particles[0].velocity;
        if(landerAlt <= 0.1 && velocity.y < 0) {
            vector<Vector3> bboxPoints(4);
            Vector3 boxMin = landerBounds.parameters[0];
            Vector3 boxMax = landerBounds.parameters[1];
            Vector3 boxSize = boxMax - boxMin;
            
            bboxPoints.push_back(boxMin);
            bboxPoints.push_back(Vector3(boxMin.x() + boxSize.x(), boxMin.y(), boxMin.z()));
            bboxPoints.push_back(Vector3(boxMin.x(), boxMin.y() + boxSize.y(), boxMin.z()));
            bboxPoints.push_back(Vector3(boxMin.x(), boxMin.y(), boxMin.z() + boxSize.z()));
            
            // Make array for contact points and call "bool Octree::checkSurfaceCollision(vector<Vector3> & bboxPoints, TreeNode & node, TreeNode & contactPoints)"
            vector<Vector3> contactPoints;
            
            // Call check surface collision for all points.
            //OPT: As soon as one of them contacts, we can leave the loop
            
            if(!landerCollide) {
                for(Vector3 boxPoint : bboxPoints) {
                    if(octree.checkSurfaceCollision(boxPoint, octree.root, contactPoints)) {
                        landerCollide = true;
                        break;
                    }
                }
            }

                
                // If there are contact points after the loop above. We know a collision has occured.
                if(contactPoints.size() > 0) {
                    // Apply collision resolution impulse force
                    float restitution = 3.0;
                    //ofVec3f norm = ofVec3f(contactPoints[0].x(), contactPoints[0].y(), contactPoints[0].z()).getNormalized();
                    ofVec3f norm = ofVec3f(0, 1, 0);
                    ofVec3f impForce = (restitution + 1.0) * ((-velocity.dot(norm)) * norm);
                    lunarModelSys->particles[0].forces += ofGetFrameRate() * impForce;
                    landerCollide = false;
                    
                }
            //}
        }
//        else
//            landerCollide = false;
//        if(lunarModelSys->particles[0].velocity.y > 0)
//            landerCollide = false;
    }
    
    //Collision Detection
    
    
    
        
    
    if(rotateCW)
        lmAngle = lmAngle - 0.75;
    if(rotateCCW)
        lmAngle = lmAngle + 0.75;
    if(rotateCW || rotateCCW)
    {
        lander.setRotation(0, lmAngle, 0, 1, 0);
    }
	
}
//--------------------------------------------------------------
void ofApp::draw(){
    // Drawing background image
    //
    if (bBackgroundLoaded) {
        ofPushMatrix();
        ofDisableDepthTest();
        ofSetColor(50, 50, 50);
        ofScale(2, 2);
        backgroundImage.draw(-200, -100);
        ofEnableDepthTest();
        ofPopMatrix();
    }
    
	theCam->begin();

	ofPushMatrix();
	if (bWireframe) {                    // wireframe mode  (include axis)
		ofDisableLighting();
		ofSetColor(ofColor::slateGray);
		terrain.drawWireframe();
		if (bLanderLoaded) {
			lander.drawWireframe();
			if (!bTerrainSelected) drawAxis(lander.getPosition());
		}
		if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
	}
	else {
		ofEnableLighting();              // shaded mode
		terrain.drawFaces();

		if (bLanderLoaded) {
			lander.drawFaces();
			if (!bTerrainSelected) drawAxis(lander.getPosition());

			ofVec3f min = lander.getSceneMin() + lander.getPosition();
			ofVec3f max = lander.getSceneMax() + lander.getPosition();

			Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));

			landerBounds = bounds;

			// set color of bounding box based on selection status
			//
			if (bLanderSelected) ofSetColor(ofColor::red);
			else ofSetColor(ofColor::white);

			drawBox(bounds);
		}
		if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
	}


	if (bDisplayPoints) {
		glPointSize(3);
		ofSetColor(ofColor::green);
		terrain.drawVertices();
	}

	// highlight selected point (draw sphere around selected point)
	//
	if (bPointSelected) {
		ofSetColor(ofColor::red);
		ofDrawSphere(selectedPoint, 2.0);
	}
	
	ofNoFill();
	ofSetColor(ofColor::white);

	// debug - check first node to make sure bbox is correct
	//
    //kdtree.draw(kdtree.root, drawLevel, 0);
    octree.draw(octree.root, drawLevel, 0);
    
    /* CODE: Draws leaf nodes and then prints out average points in each leaf.
             Used for testing.
    
    octree.drawLeafNodes(octree.root);
    if(printOnce) {
        octree.averagePointsInLeafs();
        printOnce = false;
    }
    */

    if(thrustEmitter->started)
        thrustEmitter->draw();
    
	theCam->end();
    ofDisableDepthTest();
    gui.draw();
    
    // Midterm Code
    string str;
    str += "Frame Rate: " + std::to_string(ofGetFrameRate());
    ofSetColor(ofColor::white);
    ofDrawBitmapString(str, ofGetWindowWidth() - 170, 15);

    string str2;
    //str2 += "Altitide (AGL): " + std::to_string(lander.getPosition().y);
    str2 += "Altitide (AGL): " + std::to_string(landerAlt);
    ofSetColor(ofColor::white);
    ofDrawBitmapString(str2, 5, 15);
}

// 

// Draw an XYZ axis in RGB at world (0,0,0) for reference.
//
void ofApp::drawAxis(ofVec3f location) {

	ofPushMatrix();
	ofTranslate(location);

	ofSetLineWidth(1.0);

	// X Axis
	ofSetColor(ofColor(255, 0, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(1, 0, 0));
	

	// Y Axis
	ofSetColor(ofColor(0, 255, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 1, 0));

	// Z Axis
	ofSetColor(ofColor(0, 0, 255));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 0, 1));

	ofPopMatrix();
}


void ofApp::keyPressed(int key) {

	switch (key) {
	case 'C':
	case 'c':
		if (cam.getMouseInputEnabled()) cam.disableMouseInput();
		else cam.enableMouseInput();
		break;
	case 'F':
	case 'f':
		ofToggleFullscreen();
		break;
	case 'H':
	case 'h':
		break;
	case 'r':
		cam.reset();
		break;
	case 'p':
		savePicture();
		break;
	case 't':
		setCameraTarget();
		break;
	case 'u':
		break;
	case 'v':
		togglePointsDisplay();
		break;
	case 'V':
		break;
	case 'm':
		toggleWireframeMode();
		break;
    case 'd':     // rotate spacecraft clockwise (about Y (UP) axis)
        rotateCW = true;
        break;
    case 'a':     // rotate spacecraft counter-clockwise (about Y (UP) axis)
        rotateCCW = true;
        break;
    case 'w':     // spacecraft thrust UP
        tForce->applied = false;
        thrustEmitter->start();
        tForce->set(glm::vec3(0, 0.5, 0));
        break;
    case 's':     // spacefraft thrust DOWN
        tForce->applied = false;
        thrustEmitter->start();
        tForce->set(glm::vec3(0, -0.5, 0));
        break;
	case OF_KEY_ALT:
		cam.enableMouseInput();
		bAltKeyDown = true;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = true;
		break;
	case OF_KEY_SHIFT:
		break;
	case OF_KEY_DEL:
		break;
    case OF_KEY_UP:    // move forward
        headingVector = glm::vec3(0, 0, -1);
        headingVector = glm::rotate(headingVector, glm::radians(lmAngle), glm::vec3(0, 1, 0));
        tForce->set(headingVector);
        tForce->applied = false;
        break;
    case OF_KEY_DOWN:   // move backward
        headingVector = glm::vec3(0, 0, 1);
        headingVector = glm::rotate(headingVector, glm::radians(lmAngle), glm::vec3(0, 1, 0));
        tForce->set(headingVector);
        tForce->applied = false;
        break;
    case OF_KEY_LEFT:   // move left
        headingVector = glm::vec3(-1, 0, 0);
        headingVector = glm::rotate(headingVector, glm::radians(lmAngle), glm::vec3(0, 1, 0));
        tForce->set(headingVector);
        tForce->applied = false;
        break;
    case OF_KEY_RIGHT:   // move right
        headingVector = glm::vec3(1, 0, 0);
        headingVector = glm::rotate(headingVector, glm::radians(lmAngle), glm::vec3(0, 1, 0));
        tForce->set(headingVector);
        tForce->applied = false;
        break;
	case OF_KEY_F1:
		theCam = &cam;
		break;
	case OF_KEY_F2:
		theCam = &top;
		break;
	default:
		break;
	}
}

void ofApp::toggleWireframeMode() {
	bWireframe = !bWireframe;
}

void ofApp::toggleSelectTerrain() {
	bTerrainSelected = !bTerrainSelected;
}

void ofApp::togglePointsDisplay() {
	bDisplayPoints = !bDisplayPoints;
}

void ofApp::keyReleased(int key) {

	switch (key) {
	
	case OF_KEY_ALT:
		cam.disableMouseInput();
		bAltKeyDown = false;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = false;
		break;
	case OF_KEY_SHIFT:
		break;
    case 'w':
        tForce->applied = true;
        thrustEmitter->stop();
        break;
    case 's':
        tForce->applied = true;
        thrustEmitter->stop();
        break;
    case 'd':
        rotateCW = false;
        break;
    case 'a':
        rotateCCW = false;
        break;
    case OF_KEY_UP:    // move forward
        tForce->applied = true;
        break;
    case OF_KEY_DOWN:   // move backward
        tForce->applied = true;
        break;
    case OF_KEY_LEFT:   // move left
        tForce->applied = true;
        break;
    case OF_KEY_RIGHT:   // move right
        tForce->applied = true;
        break;
	default:
		break;

	}
}



//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

	

}


//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

	// if moving camera, don't allow mouse interaction
	//
	if (cam.getMouseInputEnabled()) return;

	// if rover is loaded, test for selection
    //
    if (bLanderLoaded) {
        glm::vec3 origin = cam.getPosition();
        glm::vec3 mouseWorld = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
        glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);

        ofVec3f min = lander.getSceneMin() + lander.getPosition();
        ofVec3f max = lander.getSceneMax() + lander.getPosition();

        Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
        bool hit = bounds.intersect(Ray(Vector3(origin.x, origin.y, origin.z), Vector3(mouseDir.x, mouseDir.y, mouseDir.z)), 0, 10000);
        if (hit) {
            bLanderSelected = true;
            mouseDownPos = getMousePointOnPlane(lander.getPosition(), cam.getZAxis());
            mouseLastPos = mouseDownPos;
            bInDrag = true;
        }
        else {
            bLanderSelected = false;
        }
    }
    /* CODE: Testing intersection with surface when mouse is clicked on.
    TreeNode localNode;
    
    glm::vec3 p = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
    glm::vec3 rayDir = glm::normalize(p - theCam->getPosition());
    float timeBefore = ofGetElapsedTimef();
    float timeAfter;
    
    if(kdtree.intersect(Ray(Vector3(p.x, p.y, p.z), Vector3(rayDir.x, rayDir.y, rayDir.z)), kdtree.root, localNode)) {
        timeAfter = ofGetElapsedTimef();
        cout << "Intersect detection time in ms: " << (timeAfter - timeBefore) << endl;
        Vector3 selectedBoxCenter = localNode.box.center();
        selectedPoint = ofVec3f(selectedBoxCenter.x(), selectedBoxCenter.y(), selectedBoxCenter.z());
        bPointSelected = true;
        cout << "Location of click: " << selectedPoint << endl;
    }
    else
        cout << "The surface wasn't clicked on." << endl;
     */
}


//draw a box from a "Box" class  
//
void ofApp::drawBox(const Box &box) {
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
Box ofApp::meshBounds(const ofMesh & mesh) {
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
	return Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
}



//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

	// if moving camera, don't allow mouse interaction
    //
    if (cam.getMouseInputEnabled()) return;

    if (bInDrag) {

        glm::vec3 landerPos = lander.getPosition();

        glm::vec3 mousePos = getMousePointOnPlane(landerPos, cam.getZAxis());
        glm::vec3 delta = mousePos - mouseLastPos;
    
        landerPos += delta;
        lunarModelSys->particles[0].position = landerPos;
        //lander.setPosition(landerPos.x, landerPos.y, landerPos.z);
        mouseLastPos = mousePos;
    }

}

//  intersect the mouse ray with the plane normal to the camera
//  return intersection point.   (package code above into function)
//
glm::vec3 ofApp::getMousePointOnPlane(glm::vec3 planePt, glm::vec3 planeNorm) {
    // Setup our rays
    //
    glm::vec3 origin = cam.getPosition();
    glm::vec3 camAxis = cam.getZAxis();
    glm::vec3 mouseWorld = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
    glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
    float distance;

    bool hit = glm::intersectRayPlane(origin, mouseDir, planePt, planeNorm, distance);

    if (hit) {
        // find the point of intersection on the plane using the distance
        // We use the parameteric line or vector representation of a line to compute
        //
        // p' = p + s * dir;
        //
        glm::vec3 intersectPoint = origin + distance * mouseDir;

        return intersectPoint;
    }
    else return glm::vec3(0, 0, 0);
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
    bInDrag = false;
    bLanderSelected = false;
}



// Set the camera to use the selected point as it's new target
//  
void ofApp::setCameraTarget() {

}


//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}



//--------------------------------------------------------------
// setup basic ambient lighting in GL  (for now, enable just 1 light)
//
void ofApp::initLightingAndMaterials() {

	static float ambient[] =
	{ .5f, .5f, .5, 1.0f };
	static float diffuse[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float position[] =
	{5.0, 5.0, 5.0, 0.0 };

	static float lmodel_ambient[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float lmodel_twoside[] =
	{ GL_TRUE };


	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position);


	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
//	glEnable(GL_LIGHT1);
	glShadeModel(GL_SMOOTH);
} 

void ofApp::savePicture() {
	ofImage picture;
	picture.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
	picture.save("screenshot.png");
	cout << "picture saved" << endl;
}

//--------------------------------------------------------------
//
// support drag-and-drop of model (.obj) file loading.  when
// model is dropped in viewport, place origin under cursor
//
void ofApp::dragEvent(ofDragInfo dragInfo) {
	if (lander.loadModel(dragInfo.files[0])) {
		bLanderLoaded = true;
		lander.setScaleNormalization(false);
	//	lander.setScale(.5, .5, .5);
		lander.setPosition(0, 0, 0);
//		lander.setRotation(1, 180, 1, 0, 0);

		// We want to drag and drop a 3D object in space so that the model appears 
		// under the mouse pointer where you drop it !
		//
		// Our strategy: intersect a plane parallel to the camera plane where the mouse drops the model
		// once we find the point of intersection, we can position the lander/lander
		// at that location.
		//

		// Setup our rays
		//
		glm::vec3 origin = theCam->getPosition();
		glm::vec3 camAxis = theCam->getZAxis();
		glm::vec3 mouseWorld = theCam->screenToWorld(glm::vec3(mouseX, mouseY, 0));
		glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
		float distance;

		bool hit = glm::intersectRayPlane(origin, mouseDir, glm::vec3(0, 0, 0), camAxis, distance);
		if (hit) {
			// find the point of intersection on the plane using the distance 
			// We use the parameteric line or vector representation of a line to compute
			//
			// p' = p + s * dir;
			//
			glm::vec3 intersectPoint = origin + distance * mouseDir;

			// Now position the lander's origin at that intersection point
			//
		    glm::vec3 min = lander.getSceneMin();
			glm::vec3 max = lander.getSceneMax();
			float offset = (max.y - min.y) / 2.0;
			lander.setPosition(intersectPoint.x, intersectPoint.y - offset, intersectPoint.z);

			// set up bounding box for lander while we are at it
			//
			landerBounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
		}
	}
	

}


//  intersect the mouse ray with the plane normal to the camera 
//  return intersection point.   (package code above into function)
//
glm::vec3 ofApp::getMousePointOnPlane() {
	// Setup our rays
	//
	glm::vec3 origin = theCam->getPosition();
	glm::vec3 camAxis = theCam->getZAxis();
	glm::vec3 mouseWorld = theCam->screenToWorld(glm::vec3(mouseX, mouseY, 0));
	glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
	float distance;

	bool hit = glm::intersectRayPlane(origin, mouseDir, glm::vec3(0, 0, 0), camAxis, distance);

	if (hit) {
		// find the point of intersection on the plane using the distance 
		// We use the parameteric line or vector representation of a line to compute
		//
		// p' = p + s * dir;
		//
		glm::vec3 intersectPoint = origin + distance * mouseDir;
		
		return intersectPoint;
	}
	else return glm::vec3(0, 0, 0);
}
