#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include  "ofxAssimpModelLoader.h"
#include "Octree.h"
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
		void dragEvent2(ofDragInfo dragInfo);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void drawAxis(ofVec3f);
		void initLightingAndMaterials();
		void savePicture();
		void toggleWireframeMode();
		void togglePointsDisplay();
		void toggleSelectTerrain();
		void setCameraTarget();
		bool mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point);
		bool raySelectWithOctree(ofVec3f &pointRet);
		glm::vec3 getMousePointOnPlane(glm::vec3 p , glm::vec3 n);

		ofEasyCam cam;
        ofEasyCam shipCam;
        ofEasyCam shipTrackerCam;
		ofxAssimpModelLoader mars, lander;
		ofLight light, light1, light2;
		Box boundingBox, landerBounds, guideBox1, guideBox2;
		Box testBox;
		vector<Box> colBoxList;
		bool bLanderSelected = false;
		Octree octree;
		TreeNode selectedNode;
        TreeNode belowNode;
		glm::vec3 mouseDownPos, mouseLastPos;
		bool bInDrag = false;
        
        ParticleSystem landerSystem;
        GravityForce* gravity;
        GravityForce* normalForce;
        GravityForce* xForce;
        GravityForce* yForce;
        GravityForce* zForce;
        TurbulenceForce* explosionForce;
        TurbulenceForce* exhaustVariance;
        ImpulseRadialForce* radialForce;
    

        ofxLabel speedLabel;
        ofxLabel objectiveLabel;
        ofxLabel elevationLabel;
        ofxLabel climbLabel;
        ofxLabel fuelLabel;
		ofxIntSlider numLevels;
		ofxPanel gui;

		bool bAltKeyDown;
		bool bCtrlKeyDown;
		bool bWireframe;
		bool bDisplayPoints;
		bool bPointSelected;
		bool bHide;
		bool pointSelected = false;
		bool bDisplayLeafNodes = false;
		bool bDisplayOctree = false;
		bool bDisplayBBoxes = false;
        bool playGame = false;
		
		bool bLanderLoaded;
		bool bTerrainSelected;
        bool thrusterOn = false;
        bool forwardPressed = false;
        bool leftPressed = false;
        bool rightPressed = false;
        bool backPressed = false;
        bool spacePressed = false;
        bool shiftPressed = false;
        bool crashed = false;
    
        bool checkPt1 = false;
        bool checkPt2 = false;
	
		ofVec3f selectedPoint;
		ofVec3f intersectPoint;

		vector<Box> bboxList;

		const float selectionRange = 4.0;
    
        glm::mat4 rotXZ;
        glm::mat4 rotYZ;
    
        ofNode targetNode;
        ofEasyCam* activeCam;
        glm::vec3 landingpoint = glm::vec3(240.816, 144.992, 301.858);
        glm::vec3 flightTarget;
    
        string elevationNA = "Elevation: N/A";
        string objectiveString = "Fly to checkpoint 1";
        string tutorial = "Press p to start \nForward/backwards activate thrusters \nSpace changes pitch \nShift opens flaps \nPress s to change camera";
    
        Ray elevationRay;
    
    float thrusterTime = 0;
    bool thrustersDisabled = false;
    
    ParticleEmitter exhaust;
    ParticleEmitter explosion;
    
    ofSoundPlayer thrust;
};
