
//--------------------------------------------------------------
//
//  Kevin M. Smith
//
//  Octree Test - startup scene
// 
//
//  Student Name:   < Your Name goes Here >
//  Date: <date of last version>


#include "ofApp.h"
#include "Util.h"

#include <glm/gtx/intersect.hpp>

//--------------------------------------------------------------
// setup scene, lighting, state and load geometry
//
void ofApp::setup(){
    
    cout << tutorial << endl;
    
	bWireframe = false;
	bDisplayPoints = false;
	bAltKeyDown = false;
	bCtrlKeyDown = false;
	bLanderLoaded = false;
	bTerrainSelected = true;
//	ofSetWindowShape(1024, 768);
	cam.setDistance(20);
	cam.setNearClip(.1);
	cam.setFov(65.5);   // approx equivalent to 28mm in 35mm format
	ofSetVerticalSync(true);
	cam.disableMouseInput();
	ofEnableSmoothing();
	ofEnableDepthTest();

	// setup rudimentary lighting 
	//
	initLightingAndMaterials();

	//mars.loadModel("geo/mars-low-5x-v2.obj");
    mars.loadModel("geo/fin_terrain.obj");
	mars.setScaleNormalization(false);
    
    lander.loadModel("geo/mini_ship.obj");
    lander.setScaleNormalization(false);
    
    //lander.setPosition(0, 0, 100);
    
    if (!thrust.load("audio/George_sound.mp3"))
        ofLogFatalError("Cannot Load File George_sound.mp3");
    
    bLanderLoaded = true;

	//  Create Octree for testing.
	//
    //ofResetElapsedTimeCounter();
	octree.create(mars.getMesh(0), 20);
    //cout<< ofGetElapsedTimeMillis()<< " milliseconds to build tree"<<endl;
	cout << "Number of Verts: " << mars.getMesh(0).getNumVertices() << endl;

	testBox = Box(Vector3(3, 3, 0), Vector3(5, 5, 2));
    guideBox1 = Box(Vector3(100,700,-755), Vector3(150, 750, -745));
    guideBox2 = Box(Vector3(50,600,-255), Vector3(100, 650, -245));
    
    Particle p;
    p.lifespan = -1;
    p.damping = .999;
    p.position = glm::vec3(0,500,-1000);
    //p.position = glm::vec3(landingpoint+glm::vec3(0,500,0));
    p.rotationXZ = 0.0;
    p.rotationYZ = 0.0;
    
    lander.setPosition(0,50,0);
    
    gravity = new GravityForce(glm::vec3(0, -3.721, 0));
    //gravity = new GravityForce(glm::vec3(0, 0, 0));
    normalForce = new GravityForce(glm::vec3(0, 0, 0));
    xForce = new GravityForce(glm::vec3(0, 0, 0));
    yForce = new GravityForce(glm::vec3(0, 0, 0));
    zForce = new GravityForce(glm::vec3(0, 0, 0));
    explosionForce = new TurbulenceForce(glm::vec3(0,0, 0), glm::vec3(0,0,0));
    exhaustVariance = new TurbulenceForce(glm::vec3(0,0,0), glm::vec3(0,0,0));
    
    flightTarget = glm::vec3(guideBox1.center().x(),guideBox1.center().y(),guideBox1.center().z());
    
    landerSystem.add(p);
    landerSystem.addForce(gravity);
    landerSystem.addForce(normalForce);
    landerSystem.addForce(xForce);
    landerSystem.addForce(yForce);
    landerSystem.addForce(zForce);
    landerSystem.addForce(explosionForce);
    
    
    lander.setPosition(landerSystem.particles[0].position.x, landerSystem.particles[0].position.y, landerSystem.particles[0].position.z);
    
    //cam.setTarget(landerSystem.particles[0].position);
    //cam.setDistance(10);
    //cam.setPosition(landerSystem.particles[0].position.x, landerSystem.particles[0].position.y + 10, landerSystem.particles[0].position.z+20);
    targetNode.setPosition(landerSystem.particles[0].position);
    shipCam.setParent(targetNode);
    shipCam.setDistance(20);
    targetNode.rotateDeg(180, glm::vec3(0,1,0));
    targetNode.rotateDeg(45, glm::vec3(1,0,0));
    shipCam.disableMouseInput();
    
    shipTrackerCam.setPosition(landingpoint + glm::vec3(0, 100, 50));
    shipTrackerCam.lookAt(landerSystem.particles[0].position);
    shipTrackerCam.disableMouseInput();
    
    activeCam = &shipCam;
    
    elevationRay = Ray(Vector3(landerSystem.particles[0].position.x, landerSystem.particles[0].position.y, landerSystem.particles[0].position.z), Vector3(0,-1,0));
    
    // create sliders for testing
    //
    gui.setup();
    gui.add(numLevels.setup("Number of Octree Levels", 1, 1, 10));
    gui.add(speedLabel.setup("Speed: "+to_string(landerSystem.particles[0].velocity.length())));
    gui.add(climbLabel.setup("Climb Angle: "+to_string(-landerSystem.particles[0].rotationYZ)));
    gui.add(elevationLabel.setup(elevationNA));
    gui.add(fuelLabel.setup("Fuel: "+to_string(120-thrusterTime)));
    gui.add(objectiveLabel.setup(objectiveString));
    bHide = false;
    
    exhaust.init();
    exhaust.setPosition(glm::vec3(rotXZ*rotYZ*glm::vec4(landerSystem.particles[0].position.x, landerSystem.particles[0].position.y, landerSystem.particles[0].position.z,1)));
    exhaust.setVelocity(glm::vec3(rotXZ*rotYZ*glm::vec4(0,0,-10,1)));
    exhaust.setRate(10);
    exhaust.setGroupSize(20);
    exhaust.setLifespan(0.5);
    exhaust.setEmitterType(DirectionalEmitter);
    exhaust.sys->addForce(exhaustVariance);
    exhaust.visible = false;
    exhaust.start();
    
    explosion.setPosition(glm::vec3(rotXZ*rotYZ*glm::vec4(landerSystem.particles[0].position.x, landerSystem.particles[0].position.y, landerSystem.particles[0].position.z,1)));
    explosion.setVelocity(ofVec3f(0, 0, 0));
    explosion.setOneShot(true);
    explosion.setEmitterType(RadialEmitter);
    explosion.setGroupSize(5000);
    explosion.setParticleRadius(5);
    explosion.start();
    
    radialForce = new ImpulseRadialForce(1000.0);
    explosion.sys->addForce(radialForce);
    
    light.setup();
    light.enable();
    light.setDiffuseColor(ofColor::orange);
    light.rotateDeg(180, glm::vec3(0,0,1));
    light.setPosition(landingpoint+glm::vec3(0,1000,0));
    
    light2.setup();
    light2.enable();
    light2.setDiffuseColor(ofColor::gray);
    light2.rotateDeg(180, glm::vec3(0,0,1));
    light2.setPosition(landerSystem.particles[0].position + glm::vec3(0,10,0));
    
    light1.setup();
    light1.enable();
    light1.setDiffuseColor(ofColor::red);
    light1.rotateDeg(180, glm::vec3(0,0,1));
    light1.setPosition(shipTrackerCam.getPosition());
    light.setOrientation(shipTrackerCam.getGlobalOrientation());
}
 
//--------------------------------------------------------------
// incrementally update scene (animation)
//
void ofApp::update() {
    if(!playGame){
        //ofDrawBitmapString(tutorial, activeCam->getPosition());
        return;
    }
    
    landerSystem.update();
    
    
    elevationRay = Ray(Vector3(landerSystem.particles[0].position.x, landerSystem.particles[0].position.y, landerSystem.particles[0].position.z), Vector3(0,-1,0));
    
    light2.setPosition(shipCam.getGlobalPosition());
    light2.setOrientation(shipCam.getGlobalOrientation());
    
    light1.setOrientation(shipTrackerCam.getGlobalOrientation());
    
    if(octree.intersect(elevationRay, octree.root, belowNode)){
        elevationLabel.setup("Elevation: "+ to_string(glm::distance(glm::vec3(belowNode.box.center().x(),belowNode.box.center().y(),belowNode.box.center().z()), glm::vec3(landerSystem.particles[0].position))));
    } else {
        elevationLabel.setup(elevationNA);
    }
    
    speedLabel.setup("Speed: " + to_string(landerSystem.particles[0].velocity.length()));
    climbLabel.setup("Climb Angle :"+to_string(-landerSystem.particles[0].rotationYZ));
    fuelLabel.setup("Fuel: "+to_string(120-thrusterTime));
    objectiveLabel.setup(objectiveString);
    
    lander.setPosition(landerSystem.particles[0].position.x, landerSystem.particles[0].position.y, landerSystem.particles[0].position.z);
    
    targetNode.setPosition(landerSystem.particles[0].position);
    
    shipTrackerCam.lookAt(landerSystem.particles[0].position);
    
    //cout << landerSystem.particles[0].rotationYZ << endl;
    
    if (landerSystem.particles[0].rotationYZ < 0 && !spacePressed && !crashed){
        landerSystem.particles[0].angularForcesYZ = 5;
    }
    
    if (landerSystem.particles[0].rotationYZ > 0 && !spacePressed && !crashed){
        landerSystem.particles[0].angularForcesYZ = 0;
        landerSystem.particles[0].angularVelocityYZ = 0;
        landerSystem.particles[0].rotationYZ = 0;
    }
    
    if (crashed){
        landerSystem.particles[0].angularForcesXZ = ofRandom(-100, 100);
        landerSystem.particles[0].angularForcesYZ = ofRandom(-100, 100);;
    }
    
    ofVec3f min = lander.getSceneMin() + lander.getPosition();
    ofVec3f max = lander.getSceneMax() + lander.getPosition();

    Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
    //cout << landerSystem.particles[0].angularVelocity << endl;
    
    if(bounds.overlap(guideBox1)){
        checkPt1 = true;
        objectiveString = "Fly to checkpoint 2";
    }
    
    if(bounds.overlap(guideBox2) && checkPt1){
        checkPt2 = true;
        objectiveString = "Land in landing zone";
    }
    
    
    normalForce->set(glm::vec3(0, 0, 0));
        //float rotation = landerSystem.particles[0].rotation;
        //cout << landerSystem.particles[0].rotation <<endl;
    lander.setRotation(0, landerSystem.particles[0].rotationXZ, 0, 1, 0);
    lander.setRotation(1, landerSystem.particles[0].rotationYZ, 1, 0, 0);
    targetNode.setOrientation(glm::vec3(225+landerSystem.particles[0].rotationYZ,0,180));
    targetNode.rotateDeg(landerSystem.particles[0].rotationXZ, glm::vec3(0,1,0));
    
    rotXZ = glm::rotate(glm::mat4(1.0), glm::radians(landerSystem.particles[0].rotationXZ), glm::vec3(0, 1, 0));
    rotYZ = glm::rotate(glm::mat4(1.0), glm::radians(landerSystem.particles[0].rotationYZ), glm::vec3(1, 0, 0));
    //cout << landerSystem.particles[0].position << "; ";
    //cout << bounds.center().x() << ", " << bounds.center().y() << ", " << bounds.center().z() << ", "  <<endl;
    //cout<< targetNode.getOrientationEulerDeg()<<endl;
    //cout<< landerSystem.particles[0].velocity.length()<<", "<< landerSystem.particles[0].rotationYZ<<", "<< checkPt1 << ", "<< checkPt2<<"; "<< flightTarget <<endl;
    
    if(!checkPt1)
        flightTarget = glm::vec3(guideBox1.center().x(),guideBox1.center().y(),guideBox1.center().z());
    
    if (checkPt1 && !checkPt2)
        flightTarget = glm::vec3(guideBox2.center().x(),guideBox2.center().y(),guideBox2.center().z());
    
    if (checkPt1 && checkPt2)
        flightTarget = landingpoint;
    
    if(spacePressed && !crashed && !thrustersDisabled){
        //thrusterOn = true;
        //zForce->set(glm::vec3(rotXZ*rotYZ*glm::vec4(0,0,10,1))+glm::vec3(0,5,0));
        if (landerSystem.particles[0].rotationYZ > -90){
            landerSystem.particles[0].angularForcesYZ = -15;
        } else {
            landerSystem.particles[0].angularForcesYZ = 0;
        }
        if(landerSystem.particles[0].rotationYZ <= -90){
            landerSystem.particles[0].rotationYZ = -90;
        }
    }
    if(forwardPressed && !crashed && !thrustersDisabled){
        thrusterOn = true;
        zForce->set(glm::vec3(rotXZ*rotYZ*glm::vec4(0,0,10,1))+glm::vec3(0,3.721,0));
        
    }
    if(leftPressed && !crashed&& !thrustersDisabled){
        thrusterOn = true;
        zForce->set(glm::vec3(rotXZ*rotYZ*glm::vec4(2.5,0,5,1)));
        landerSystem.particles[0].angularForcesXZ = 15;
        /*for(float i = 0; i <= 15; i+=0.1){
            lander.setRotation(2, i, 0, 1, 0);
        }*/
    }
    if(rightPressed && !crashed && !thrustersDisabled){
        thrusterOn = true;
        zForce->set(glm::vec3(rotXZ*rotYZ*glm::vec4(-2.5,0,5,1)));
        landerSystem.particles[0].angularForcesXZ = -15;
        /*for(float i = 0; i <= 15; i+=0.1){
            lander.setRotation(2, -i, 0, 1, 0);
        }*/
    }
    if(backPressed && !crashed && !thrustersDisabled){
        thrusterOn = true;
        zForce->set(glm::vec3(rotXZ*rotYZ*glm::vec4(0,0,-10,1)));
    }
    if(shiftPressed && !crashed){
        landerSystem.particles[0].velocity = glm::vec3(landerSystem.particles[0].velocity.x*.99, landerSystem.particles[0].velocity.y, landerSystem.particles[0].velocity.z*.99);
        //thrusterOn = true;
        //landerSystem.particles[0].angularForcesYZ = -15;
        //yForce->set(glm::vec3(0,5,0));
        //if(landerSystem.particles[0].rotationYZ <= -90){
            //landerSystem.particles[0].rotationYZ = -90;
        //}
    }
    
    if(thrusterOn){
        thrusterTime += 1/ofGetFrameRate();
        thrust.play();
        exhaust.update();
        exhaustVariance->set(glm::vec3(rotXZ*rotYZ*glm::vec4(-20,-20,0,1)), glm::vec3(rotXZ*rotYZ*glm::vec4(20,20,0,1)));
        exhaust.setPosition(landerSystem.particles[0].position + rotXZ*rotYZ*glm::vec4(0,0.5,-4.5,1));
        exhaust.setVelocity(landerSystem.particles[0].velocity+glm::vec3(rotXZ*rotYZ*glm::vec4(0,0,-5,1)));
    }
        //cout<<thrusterTime<<endl;
    if (thrusterTime >= 120){
        thrustersDisabled = true;
    }
    
    if(octree.intersect(bounds, octree.root, colBoxList)){
        //cout << landerSystem.particles[0].velocity << endl;
        
        float finalVelocity =landerSystem.particles[0].velocity.length();
        cout << finalVelocity <<endl;
        
        if(!checkPt1){
            cout<<"Missed first checkpoint"<<endl;
        }
        if(!checkPt2){
            cout<<"Missed second checkpoint"<<endl;
        }
        
        if(landerSystem.particles[0].rotationYZ < -80 && landerSystem.particles[0].rotationYZ > -100 && !crashed){
            cout << "Upright" << endl;
        }
        if(glm::distance(glm::vec3(landerSystem.particles[0].position), landingpoint) <= 20 && !crashed){
            cout<<"Landed in landing zone"<<endl;
        }
        if(finalVelocity > 10 || crashed){
            cout<<"Hit the ground too hard" << endl;
            crashed = true;
            explosion.setPosition(landerSystem.particles[0].position);
            explosionForce->set(glm::vec3(-100,-100, -100), glm::vec3(100,100,100));
            //explosion.start();
            explosion.update();
        } else if(!crashed) {
            playGame = false;
        }
        
    }
}
//--------------------------------------------------------------
void ofApp::draw() {

	ofBackground(ofColor::black);
    
    light.draw();
    light1.draw();
    light2.draw();

	glDepthMask(false);
	if (!bHide) gui.draw();
	glDepthMask(true);

	activeCam->begin();
	ofPushMatrix();
	if (bWireframe) {                    // wireframe mode  (include axis)
		ofDisableLighting();
		ofSetColor(ofColor::slateGray);
		mars.drawWireframe();
		if (bLanderLoaded) {
			lander.drawWireframe();
			if (!bTerrainSelected) drawAxis(lander.getPosition());
		}
		if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
	}
	else {
		ofEnableLighting();              // shaded mode
		mars.drawFaces();
		ofMesh mesh;
		if (bLanderLoaded) {
			lander.drawFaces();
			if (!bTerrainSelected) drawAxis(lander.getPosition());
			if (bDisplayBBoxes) {
				ofNoFill();
				ofSetColor(ofColor::white);
				for (int i = 0; i < lander.getNumMeshes(); i++) {
					ofPushMatrix();
					ofMultMatrix(lander.getModelMatrix());
					ofRotate(-90, 1, 0, 0);
					Octree::drawBox(bboxList[i]);
					ofPopMatrix();
				}
			}

			if (bLanderSelected) {

				ofVec3f min = lander.getSceneMin() + lander.getPosition();
				ofVec3f max = lander.getSceneMax() + lander.getPosition();

				Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
				ofSetColor(ofColor::white);
				Octree::drawBox(bounds);

				// draw colliding boxes
				//
				ofSetColor(ofColor::lightBlue);
				for (int i = 0; i < colBoxList.size(); i++) {
					Octree::drawBox(colBoxList[i]);
				}
			}
		}
	}
	if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));

    //Octree::drawBox(testBox);

	if (bDisplayPoints) {                // display points as an option    
		glPointSize(3);
		ofSetColor(ofColor::green);
		mars.drawVertices();
	}

	// highlight selected point (draw sphere around selected point)
	//
	if (bPointSelected) {
		ofSetColor(ofColor::blue);
		ofDrawSphere(selectedPoint, .1);
	}


	// recursively draw octree
	//
	ofDisableLighting();
	int level = 0;
	//	ofNoFill();

	if (bDisplayLeafNodes) {
		octree.drawLeafNodes(octree.root);
		cout << "num leaf: " << octree.numLeaf << endl;
    }
	else if (bDisplayOctree) {
		ofNoFill();
		ofSetColor(ofColor::white);
		octree.draw(numLevels, 0);
	}

	// if point selected, draw a sphere
	//
	if (pointSelected) {
		ofVec3f p = octree.mesh.getVertex(selectedNode.points[0]);
		ofVec3f d = p - cam.getPosition();
		ofSetColor(ofColor::lightGreen);
		ofDrawSphere(p, .02 * d.length());
	}
    
    //targetNode.draw();
    //shipCam.draw();
    ofDrawArrow(landerSystem.particles[0].position, landerSystem.particles[0].position + glm::vec3(0,5,0));
    ofSetColor(ofColor::green);
    ofDrawArrow(landerSystem.particles[0].position, landerSystem.particles[0].position + glm::vec3(rotXZ*glm::vec4(0,0,7.5,1)));
    ofSetColor(ofColor::red);
    if(glm::distance(glm::vec3(landingpoint.x, 0, landingpoint.z), glm::vec3(landerSystem.particles[0].position.x,0,landerSystem.particles[0].position.z)) <=20)
        ofSetColor(ofColor::yellow);
    
    ofDrawArrow(landerSystem.particles[0].position, flightTarget);
    
    ofNoFill();
    ofDrawCylinder(landingpoint + glm::vec3(0,125,0), 20, 240);
    //ofDrawSphere(landingpoint, 20);
    
    ofSetColor(ofColor::white);
    
    if(!checkPt1)
        octree.drawBox(guideBox1);
    if(!checkPt2)
        octree.drawBox(guideBox2);
    
    shipTrackerCam.draw();
    exhaust.draw();
    
	ofPopMatrix();
    
    explosion.draw();
    
	activeCam->end();
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
	case 'B':
	case 'b':
		bDisplayBBoxes = !bDisplayBBoxes;
		break;
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
	case 'L':
	case 'l':
		bDisplayLeafNodes = !bDisplayLeafNodes;
		break;
	case 'O':
	case 'o':
		bDisplayOctree = !bDisplayOctree;
		break;
    case 'p':
        playGame = !playGame;
        break;
	case 'r':
        landerSystem.particles[0].position = glm::vec3(0,500,-1000);
            
            playGame = true;
            thrusterTime=0;
            thrustersDisabled=false;
            crashed=false;
            checkPt1=false;
            checkPt2=false;
            explosionForce->set(glm::vec3(0,0,0), glm::vec3(0,0,0));
		break;
	case 's':
        if (activeCam == &cam) activeCam = &shipCam;
        else if (activeCam == &shipCam) activeCam = &shipTrackerCam;
        else if (activeCam == &shipTrackerCam) activeCam = &cam;
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
	case 'w':
		toggleWireframeMode();
		break;
	case OF_KEY_ALT:
		cam.enableMouseInput();
		bAltKeyDown = true;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = true;
		break;
	case OF_KEY_SHIFT:
            shiftPressed = true;
		break;
	case OF_KEY_DEL:
		break;
    case ' ':
            
            spacePressed = true;
            /*cout << landerSystem.particles[0].position<< endl;
            zForce->set(glm::vec3(rotXZ*rotYZ*glm::vec4(0,0,5,1))+glm::vec3(0,5,0));
            if (landerSystem.particles[0].rotationYZ > -45){
                landerSystem.particles[0].angularForcesYZ = -50;
            } else {
                landerSystem.particles[0].angularForcesYZ = 0;
            }*/
            break;
        break;
    case OF_KEY_UP:
        
        forwardPressed = true;
        //zForce->set(glm::vec3(rotXZ*rotYZ*glm::vec4(0,0,5,1)));
        break;
    case OF_KEY_LEFT:
        
            leftPressed = true;
            /*zForce->set(glm::vec3(rotXZ*rotYZ*glm::vec4(0,0,5,1)));
        landerSystem.particles[0].angularForcesXZ = 50;
        for(int i = 0; i <= 15; i++){
            lander.setRotation(2, i, 0, 1, 0);
        }*/
        break;
    case OF_KEY_DOWN:
            backPressed = true;
            
            //landerSystem.particles[0].damping = .75;
            //zForce->set(glm::vec3(rotXZ*rotYZ*glm::vec4(0,0,-5,1)));
        //zForce->set(glm::vec3(0,0,-5));
        break;
    case OF_KEY_RIGHT:
            
            rightPressed = true;
            /*zForce->set(glm::vec3(rotXZ*rotYZ*glm::vec4(0,0,5,1)));
        landerSystem.particles[0].angularForcesXZ = -50;
        for(int i = 0; i <= 15; i++){
            lander.setRotation(2, -i, 0, 1, 0);
        }*/
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
            thrusterOn = false;
            shiftPressed = false;
            landerSystem.particles[0].angularVelocityYZ = 0;
            landerSystem.particles[0].angularAccelerationYZ = 0;
            landerSystem.particles[0].angularForcesYZ = 15;
		break;
    case ' ':
            //thrusterOn = false;
            spacePressed=false;
            zForce->set(glm::vec3(0,0,0));
            landerSystem.particles[0].angularVelocityYZ = 0;
            landerSystem.particles[0].angularAccelerationYZ = 0;
            landerSystem.particles[0].angularForcesYZ = 15;
            break;
        break;
    case OF_KEY_UP:
        thrusterOn = false;
            forwardPressed=false;
        zForce->set(glm::vec3(0,0,0));
        //landerSystem.particles[0].angularForcesYZ = 0;
        break;
    case OF_KEY_LEFT:
            thrusterOn = false;
            leftPressed=false;
            zForce->set(glm::vec3(0,0,0));
            landerSystem.particles[0].angularVelocityXZ = 0;
            landerSystem.particles[0].angularAccelerationXZ = 0;
        landerSystem.particles[0].angularForcesXZ = 0;
        for(float i = 0; i <= 15; i+=0.1){
                lander.setRotation(2, -15+i, 0, 1, 0);
        }
        break;
    case OF_KEY_DOWN:
            backPressed = false;
            thrusterOn = false;
            zForce->set(glm::vec3(0,0,0));
            
        break;
    case OF_KEY_RIGHT:
            rightPressed=false;
            thrusterOn = false;
            zForce->set(glm::vec3(0,0,0));
            landerSystem.particles[0].angularVelocityXZ = 0;
            landerSystem.particles[0].angularAccelerationXZ = 0;
        landerSystem.particles[0].angularForcesXZ = 0;
        for(float i = 0; i <= 15; i+=0.1){
            lander.setRotation(2, 15-i, 0, 1, 0);
        }
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
	else {
		ofVec3f p;
        //ofResetElapsedTimeCounter();
		raySelectWithOctree(p);
        //cout <<  p << endl;
	}
}

bool ofApp::raySelectWithOctree(ofVec3f &pointRet) {
	ofVec3f mouse(mouseX, mouseY);
	ofVec3f rayPoint = cam.screenToWorld(mouse);
	ofVec3f rayDir = rayPoint - cam.getPosition();
	rayDir.normalize();
	Ray ray = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z),
		Vector3(rayDir.x, rayDir.y, rayDir.z));

	pointSelected = octree.intersect(ray, octree.root, selectedNode);

	if (pointSelected) {
		pointRet = octree.mesh.getVertex(selectedNode.points[0]);
	}
	return pointSelected;
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
		lander.setPosition(landerPos.x, landerPos.y, landerPos.z);
		mouseLastPos = mousePos;
        landerSystem.particles[0].position.set(lander.getPosition());
		ofVec3f min = lander.getSceneMin() + lander.getPosition();
		ofVec3f max = lander.getSceneMax() + lander.getPosition();
        
		Box bounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));

		colBoxList.clear();
		octree.intersect(bounds, octree.root, colBoxList);
	

		/*if (bounds.overlap(testBox)) {
			cout << "overlap" << endl;
		}
		else {
			cout << "OK" << endl;
		}*/


	}
	else {
		ofVec3f p;
        //ofResetElapsedTimeCounter();
        raySelectWithOctree(p);
        //cout << ofGetElapsedTimeMillis() << " milliseconds to search point at "<< p << endl;
        cout <<  p << endl;
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
	bInDrag = false;
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
void ofApp::dragEvent2(ofDragInfo dragInfo) {

	ofVec3f point;
	mouseIntersectPlane(ofVec3f(0, 0, 0), cam.getZAxis(), point);
	/*if (lander.loadModel(dragInfo.files[0])) {
		lander.setScaleNormalization(false);
//		lander.setScale(.1, .1, .1);
	//	lander.setPosition(point.x, point.y, point.z);
		lander.setPosition(1, 1, 0);

		bLanderLoaded = true;
		for (int i = 0; i < lander.getMeshCount(); i++) {
			bboxList.push_back(Octree::meshBounds(lander.getMesh(i)));
		}

		cout << "Mesh Count: " << lander.getMeshCount() << endl;
	}
	else cout << "Error: Can't load model" << dragInfo.files[0] << endl;*/
}

bool ofApp::mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point) {
	ofVec2f mouse(mouseX, mouseY);
	ofVec3f rayPoint = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
	ofVec3f rayDir = rayPoint - cam.getPosition();
	rayDir.normalize();
	return (rayIntersectPlane(rayPoint, rayDir, planePoint, planeNorm, point));
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
		lander.setPosition(0, 0, 0);
		cout << "number of meshes: " << lander.getNumMeshes() << endl;
		bboxList.clear();
		for (int i = 0; i < lander.getMeshCount(); i++) {
			bboxList.push_back(Octree::meshBounds(lander.getMesh(i)));
		}

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
		glm::vec3 origin = cam.getPosition();
		glm::vec3 camAxis = cam.getZAxis();
		glm::vec3 mouseWorld = cam.screenToWorld(glm::vec3(mouseX, mouseY, 0));
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
