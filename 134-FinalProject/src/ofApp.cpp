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
    
    bgSound.load("sounds/space.wav");
    
    thrust.load("sounds/rocket-thrust-01.wav");
    bgSound.setVolume(0.3);
    bgSound.play();

	top.setNearClip(.1);
	top.setFov(80);
    
    traceCam.setPosition(0, 1, 130);
    traceCam.setNearClip(.1);
    traceCam.setFov(40);

	theCam = &cam;

	// setup rudimentary lighting 
	//
	initLightingAndMaterials();

    terrain.loadModel("geo/moon-houdini.obj");
	terrain.setScaleNormalization(false);

	boundingBox = meshBounds(terrain.getMesh(0));
    
    // Texture loading
    ofDisableArbTex();     // disable rectangular textures
    
    // Load Textures
    if (!ofLoadImage(particleTex, "images/dot.png")) {
        cout << "Particle Texture File: images/dot.png not found" << endl;
        ofExit();
    }

    // Load the Shaders
    #ifdef TARGET_OPENGLES
        shader.load("shaders_gles/shader");
    #else
        shader.load("shaders/shader");
    #endif

	// create KdTree for terrain
	//
    float timeBefore = ofGetElapsedTimef();
	//kdtree.create(terrain.getMesh(0), 40);
    octree.create(terrain.getMesh(0), 8);
    float timeAfter = ofGetElapsedTimef();
    cout << "Time taken to build tree in MS: " << (timeAfter - timeBefore) << endl;
    
    gui.setup();
    gui.add(drawLevel.setup("Draw Level", 1, 1, 7));
    //gui.add(camFOV.setup("FOV", 65.5, 0, 100));
    //gui.add(camNC.setup("Near Clip", .1, 0, 10));
    
    // Midterm Code
    
    if (lander.loadModel("geo/rocket2.obj")) {
        lander.setScaleNormalization(false);
        //lander.setScale(.5, .5, .5);
        //lander.setRotation(0, -180, 1, 0, 0);
        lander.setPosition(0, 15, 0);
        bLanderLoaded = true;
    }
    else {
        cout << "Error: Can't load model" << "geo/lander.obj" << endl;
        ofExit(0);
    }
    
    lmAngle = 0;
    headingVector = glm::vec3(0, 0, 0);
    
    // Setup LEM
    lunarModel = new Particle();
    lunarModelSys = new ParticleSystem();
    
    lunarModel->position = glm::vec3(0, 5, 0);
    lunarModel->radius = 0.25;
    lunarModel->lifespan = -1;
    lunarModelSys->add(*lunarModel);
    tForce = new ThrusterForce(glm::vec3(0, 0, 0));
    tForce->applied = true;
    lunarModelSys->addForce(tForce);
    lunarModelSys->addForce(new TurbulenceForce(glm::vec3(-0.3, -0.3, -0.3), glm::vec3(0.3, 0.3, 0.3)));
    lunarModelSys->addForce(new GravityForce(ofVec3f(-0.3)));
    
    thrustEmitter = new ParticleEmitter();
    thrustEmitter->type = DiscEmitter;
    thrustEmitter->velocity = glm::vec3(0, -3, 0);
    thrustEmitter->rate = 20;
    thrustEmitter->randomLife = true;
    thrustEmitter->lifeMinMax = ofVec3f(0.15, 0.45);
    thrustEmitter->radius = 0.20;
    thrustEmitter->particleRadius = 5;
    thrustEmitter->groupSize = 100;
    
    keyLight.setup();
    keyLight.enable();
    keyLight.setAreaLight(2, 2);
    keyLight.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
    keyLight.setDiffuseColor(ofFloatColor(1, 1, 1));
    keyLight.setSpecularColor(ofFloatColor(1, 0, 0));

    //keyLight.rotate(45, ofVec3f(0, 1, 0));
    keyLight.rotate(180, ofVec3f(1, 0, 0));
    keyLight.setPosition(lander.getPosition());
    
    ofVec3f pos = lander.getPosition();
    fillLight.setup();
    fillLight.enable();
    fillLight.setSpotlight();
    fillLight.setScale(.1);
    fillLight.setSpotlightCutOff(200);
    fillLight.setAttenuation(2, .00001, .00001);
    fillLight.setAmbientColor(ofFloatColor(0.1, 0, 0));
    fillLight.setDiffuseColor(ofFloatColor(1, 1, 1));
    fillLight.setSpecularColor(ofFloatColor(0, 1, 0));
    fillLight.rotate(-90, ofVec3f(1, 0, 0));
    //fillLight.rotate(-90, ofVec3f(0, 1, 0));
    fillLight.setPosition(pos.x,pos.y + 10,pos.z);
    
    explosions = new ParticleEmitter();
    explosions->type = RadialEmitter;
    explosions->particleColor = ofColor::yellow;
    explosions->visible = false;
    explosions->velocity = ofVec3f(3, 0, 3);
    explosions->setOneShot(true);
    explosions->setEmitterType(RadialEmitter);
    explosions->setParticleRadius(0.02);
    explosions->setLifespan(0.8);
    explosions->setGroupSize(200);
    
    //Landing Area Setup
    landAreaWidth = 30;
    landAreaHeight = 30;
    landAreaCoords.push_back(ofVec3f(100.0, -0.5, -131.0));
    landAreaCoords.push_back(ofVec3f(-150, -0.5, 84.0));
    landAreaCoords.push_back(ofVec3f(89.0, -0.5, 92.0));
    
    for(int i = 0; i < landAreaCoords.size(); i++) {
        ofPolyline landingSquare;
        landingSquare.addVertex(landAreaCoords[i]);
        landingSquare.addVertex(ofVec3f(landAreaCoords[i].x, landAreaCoords[i].y, landAreaCoords[i].z + landAreaWidth));
        landingSquare.addVertex(ofVec3f(landAreaCoords[i].x + landAreaHeight, landAreaCoords[i].y, landAreaCoords[i].z + landAreaWidth));
        landingSquare.addVertex(ofVec3f(landAreaCoords[i].x + landAreaHeight, landAreaCoords[i].y, landAreaCoords[i].z));
        landingSquare.addVertex(landAreaCoords[i]);
        landAreaPolys.push_back(landingSquare);
    }
    
    landedAreas.push_back(false);
    landedAreas.push_back(false);
    landedAreas.push_back(false);
    
    ofTrueTypeFont::setGlobalDpi(72);
    verdana44.load("verdana.ttf", 44, true, true);
    verdana22.load("verdana.ttf", 22, true, true);
}

// Load vertex buffer in preparation for rendering
void ofApp::loadVbo() {
    if (thrustEmitter->sys->particles.size() < 1) return;

    vector<ofVec3f> sizes;
    vector<ofVec3f> points;
    for (int i = 0; i < thrustEmitter->sys->particles.size(); i++) {
        points.push_back(thrustEmitter->sys->particles[i].position);
        sizes.push_back(ofVec3f(thrustEmitter->particleRadius));
    }
    // upload the data to the vbo
    int total = (int)points.size();
    vbo.clear();
    vbo.setVertexData(&points[0], total, GL_STATIC_DRAW);
    vbo.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
}

//--------------------------------------------------------------
// incrementally update scene (animation)
//
void ofApp::update() {
    gameWon = checkGameWon();
    
    if(fuel <= 0) gameOver = true;
    
    ofVec3f pos = lander.getPosition();
    explosions->update();
    explosions->setPosition(ofVec3f(pos.x,pos.y,pos.z));
    
    if(!(gameOver || gameWon)) {
    lunarModelSys->update();
    lander.setPosition(lunarModelSys->particles[0].position.x, lunarModelSys->particles[0].position.y, lunarModelSys->particles[0].position.z);
    
    keyLight.setPosition(pos.x,pos.y - 100,pos.z);
    fillLight.setPosition(pos.x,pos.y + 40,pos.z);
    //fillLight2.setPosition(pos.x,pos.y,pos.z);
    
    thrustEmitter->update();
    thrustEmitter->setPosition(glm::vec3(lunarModelSys->particles[0].position.x, lunarModelSys->particles[0].position.y, lunarModelSys->particles[0].position.z));
    
    if(bLanderLoaded) {
        // Altitude Detection
        Vector3 origin = Vector3(lander.getPosition().x, lander.getPosition().y, lander.getPosition().z);
        Vector3 direction = Vector3(0, -1, 0);
        Ray altDetect = Ray(origin, direction);
        TreeNode localNode;
        
        if(octree.intersect(altDetect, octree.root, localNode)) {
            landerAlt = lunarModelSys->particles[0].position.y - localNode.box.center().y();
        }
        
        // Collision Detection
        ofVec3f landerVel = lunarModelSys->particles[0].velocity;
        
        if(landerVel.y < -0.03) {
            float stepSize = ofGetFrameRate();
            ofVec3f distancePerFrame = -landerVel + (1.0 / stepSize);
            
            if(distancePerFrame.y > landerAlt && landerAlt <= 0.1) {
                landerCollide = true;
                vector<Vector3> bboxPoints(4);
                Vector3 boxMin = landerBounds.parameters[0];
                Vector3 boxMax = landerBounds.parameters[1];
                Vector3 boxSize = boxMax - boxMin;
                bboxPoints.push_back(boxMin);
                bboxPoints.push_back(Vector3(boxMin.x() + boxSize.x(), boxMin.y(), boxMin.z()));
                bboxPoints.push_back(Vector3(boxMin.x(), boxMin.y() + boxSize.y(), boxMin.z()));
                bboxPoints.push_back(Vector3(boxMin.x(), boxMin.y(), boxMin.z() + boxSize.z()));
                
                vector<Vector3> contactPoints;
                
                if(landerCollide) {
                    for(Vector3 boxPoint : bboxPoints) {
                        octree.checkSurfaceCollision(boxPoint, octree.root, contactPoints);
                        if(contactPoints.size() > 0) {
                            float restitution = 1;
                            ofVec3f norm = ofVec3f(0, 1, 0);
                            ofVec3f impForce = (restitution + 0.5) * ((-landerVel.dot(norm)) * norm);
                            lunarModelSys->particles[0].forces += ofGetFrameRate() * impForce;
                            
                            float scoreReturned = checkLandingVelocity(landerVel.y);
                            bool insideLA = checkInsideLandingAreas(lunarModelSys->particles[0].position);
                            
                            if(insideLA && scoreReturned != 0) {
                                cout << "Landed in a landing area!" << endl;
                                gameScore += scoreReturned;
                                fuel += 50.0;
                                cout << "Player score: " << scoreReturned << endl;
                            }
                            else {
                                if(insideLA && scoreReturned == 0)
                                    cout << "Landed inside landing area, but impact too hard!" << endl;
                                else
                                    cout << "Landed outside landing area!" << endl;
                                explosions->start();
                                gameOver = true;
                            }
                            landerCollide = false;
                            cout << "IFA, Lander y vel:" << lunarModelSys->particles[0].velocity.y << endl;
                            break;
                        }
                    }
                }
            }
        }
        else
            landerCollide = false;
    }
    }
    
    // Update Cameras
    top.setPosition(lunarModelSys->particles[0].position.x, lunarModelSys->particles[0].position.y - 3, lunarModelSys->particles[0].position.z);
    top.lookAt(glm::vec3(lunarModelSys->particles[0].position.x, 0, lunarModelSys->particles[0].position.z));
    
    traceCam.lookAt(lunarModelSys->particles[0].position);
    
    if(rotateCW)
    {
        lmAngle = lmAngle - 0.75;
        fillLight.rotate(-0.75, ofVec3f(0, 1, 0));
        //fillLight2.rotate(-0.75, ofVec3f(0, 1, 0));
        
    }
    if(rotateCCW)
    {
        lmAngle = lmAngle + 0.75;
        fillLight.rotate(0.75, ofVec3f(0, 1, 0));
        //fillLight2.rotate(0.75, ofVec3f(0, 1, 0));
    }
    if(rotateCW || rotateCCW)
        lander.setRotation(0, lmAngle, 0, 1, 0);
}

bool ofApp::checkGameWon() {
    for(int i = 0; i < landedAreas.size(); i++) {
        if (!landedAreas[i])
            return false;
    }
    return true;
}

bool ofApp::checkInsideLandingAreas(ofVec3f landerPos) {
    for(int i = 0; i < landAreaCoords.size(); i++) {
        if(landerPos.x >= landAreaCoords[i].x && landerPos.x <= landAreaCoords[i].x + landAreaHeight &&
           landerPos.z >= landAreaCoords[i].z && landerPos.z <= landAreaCoords[i].z + landAreaWidth) {
            landedAreas[i] = true;
            return true;
        }
    }
    return false;
}

float ofApp::checkLandingVelocity(float landingVel) {
    landingVel = -landingVel;
    if(landingVel >= 1.0) {
        gameOver = true;
        return 0;
    }
    else {
        return (100.0 - (landingVel * 100));
    }
}


//--------------------------------------------------------------
void ofApp::draw(){
    // Drawing background image
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
        keyLight.draw();
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
    
    // Draw the landing regions
    ofNoFill();
    ofSetColor(0, 230, 0);
    for(int i = 0; i < landAreaPolys.size(); i++) {
        if(landedAreas[i])
             ofSetColor(255, 0, 0);
        else
             ofSetColor(0, 230, 0);
        landAreaPolys[i].draw();
        for(int j = 0; j < landAreaPolys[i].getVertices().size() -1; j++) {
            ofDrawSphere(landAreaPolys[i].getVertices()[j], 0.7);
        }
    }

	ofSetColor(ofColor::white);

	// debug - check first node to make sure bbox is correct
	//
    octree.draw(octree.root, drawLevel, 0);
    
    /* CODE: Draws leaf nodes and then prints out average points in each leaf.
             Used for testing.
    
    octree.drawLeafNodes(octree.root);
    if(printOnce) {
        octree.averagePointsInLeafs();
        printOnce = false;
    }
     */
    
    explosions->draw();
	theCam->end();
    ofDisableDepthTest();
    gui.draw();
    
    if(thrustEmitter->started) {
        loadVbo();
        glDepthMask(GL_FALSE);

        ofSetColor(255, 100, 90);

        // this makes everything look glowy :)
        //
        ofEnableBlendMode(OF_BLENDMODE_ADD);
        ofEnablePointSprites();

        // begin drawing in the camera
        shader.begin();
        theCam->begin();

        // draw particle emitter here..
        //emitter.draw();
        particleTex.bind();
        vbo.draw(GL_POINTS, 0, (int)thrustEmitter->sys->particles.size());
        particleTex.unbind();

        //  end drawing in the camera
        //
        theCam->end();
        shader.end();

        ofDisablePointSprites();
        ofDisableBlendMode();
        ofEnableAlphaBlending();

        // set back the depth mask
        //
        glDepthMask(GL_TRUE);
    }
    
    if(gameOver) {
        ofSetColor(255, 0, 0);
        verdana44.drawString("GAME OVER!", ofGetWindowWidth() / 2 - 60, ofGetWindowHeight() / 2 - 30);
        verdana22.drawString("Score:" + std::to_string(static_cast<int>(gameScore)), ofGetWindowWidth() / 2 - 60, ofGetWindowHeight() / 2 - 5);
    }
    if(gameWon) {
        ofSetColor(0, 230, 0);
        verdana44.drawString("LANDING COMPLETE!", ofGetWindowWidth() / 2 - 60, ofGetWindowHeight() / 2 - 30);
        verdana22.drawString("Score:" + std::to_string(static_cast<int>(gameScore)), ofGetWindowWidth() / 2 - 60, ofGetWindowHeight() / 2 - 5);
    }
    
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
    
    string str3;
    str3 += "Fuel: " + std::to_string(fuel);
    ofSetColor(ofColor::white);
    ofDrawBitmapString(str3, ofGetWindowWidth() - 170, 30);
    ofSetVerticalSync(true);
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
        //cam.lookAt(lunarModelSys->particles[0].position);
        //cam.setPosition(lunarModelSys->particles[0].position + ofVec3f(50, 50, 50));
        cam.setDistance(15);
        cam.setTarget(lunarModelSys->particles[0].position);
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
        if(!thrustOn){
            thrustOn = true;
            thrust.play();
        }
            fuel -= 0.1;
        tForce->set(glm::vec3(0, 1.5, 0));
        break;
    case 's':     // spacefraft thrust DOWN
        tForce->applied = false;
        thrustEmitter->start();
        if(!thrustOn){
            thrustOn = true;
            thrust.play();
        }
            fuel -= 0.1;
        tForce->set(glm::vec3(0, -1.5, 0));
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
        headingVector = glm::vec3(0, 0, -5);
        headingVector = glm::rotate(headingVector, glm::radians(lmAngle), glm::vec3(0, 1, 0));
        tForce->set(headingVector);
            fuel -= 0.1;
        tForce->applied = false;
        break;
    case OF_KEY_DOWN:   // move backward
        headingVector = glm::vec3(0, 0, 5);
        headingVector = glm::rotate(headingVector, glm::radians(lmAngle), glm::vec3(0, 1, 0));
        tForce->set(headingVector);
            fuel -= 0.1;
        tForce->applied = false;
        break;
    case OF_KEY_LEFT:   // move left
        headingVector = glm::vec3(-5, 0, 0);
        headingVector = glm::rotate(headingVector, glm::radians(lmAngle), glm::vec3(0, 1, 0));
        tForce->set(headingVector);
            fuel -= 0.1;
        tForce->applied = false;
        break;
    case OF_KEY_RIGHT:   // move right
        headingVector = glm::vec3(5, 0, 0);
        headingVector = glm::rotate(headingVector, glm::radians(lmAngle), glm::vec3(0, 1, 0));
        tForce->set(headingVector);
            fuel -= 0.1;
        tForce->applied = false;
        break;
	case OF_KEY_F1:
		theCam = &cam;
		break;
	case OF_KEY_F2:
		theCam = &top;
		break;
    case OF_KEY_F3:
        theCam = &traceCam;
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
        thrust.stop();
        thrustOn = false;
        break;
    case 's':
        tForce->applied = true;
        thrustEmitter->stop();
        thrust.stop();
        thrustOn = false;
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
    
    if(octree.intersect(Ray(Vector3(p.x, p.y, p.z), Vector3(rayDir.x, rayDir.y, rayDir.z)), octree.root, localNode)) {
        timeAfter = ofGetElapsedTimef();
        cout << "Intersect detection time in ms: " << (timeAfter - timeBefore) << endl;
        Vector3 selectedBoxCenter = localNode.box.center();
        selectedPoint = ofVec3f(selectedBoxCenter.x(), selectedBoxCenter.y(), selectedBoxCenter.z());
        cout << "Selected points coordinators: " << selectedPoint << endl;
        
        bPointSelected = true;
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
	glEnable(GL_LIGHT1);
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
