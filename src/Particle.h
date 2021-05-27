#pragma once

#include "ofMain.h"

class ParticleForceField;

class Particle {
public:
	Particle();

	ofVec3f position = ofVec3f(0,0,0);
	ofVec3f velocity = ofVec3f(0,0,0);
	ofVec3f acceleration= ofVec3f(0,0,0);
	ofVec3f forces = ofVec3f(0,0,0);
    float   rotationXZ = 0;
    float   angularVelocityXZ = 0;
    float   angularAccelerationXZ = 0;
    float   angularForcesXZ = 0;
    float   rotationYZ = 0;
    float   angularVelocityYZ = 0;
    float   angularAccelerationYZ = 0;
    float   angularForcesYZ = 0;
	float	damping;
	float   mass;
	float   lifespan;
	float   radius;
	float   birthtime;
	void    integrate();
	void    draw();
	float   age();        // sec
	ofColor color;
};


