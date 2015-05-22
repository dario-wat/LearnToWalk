#include "robot.hpp"
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <iostream>
#include <cstdlib>

#ifdef dDOUBLE
#define dsDrawCapsule 	dsDrawCapsuleD
#define dsDrawBox     	dsDrawBoxD
#define dsDrawLine    	dsDrawLineD
#define dsDrawCylinder	dsDrawCylinderD
#endif


// RGB color values [0..1]
static const double TORSO_COLOR[3] =	{0.9, 0.2, 0.2};
static const double LEG_COLOR[3] =		{0.2, 0.9, 0.2};
static const double SENSOR_COLOR[3] =	{0.2, 0.2, 0.9};

// Masses of body parts
static const dReal TORSO_M = 10.0;		// torso mass
static const dReal SENSOR_M = 0.01;		// sensors mass
static const dReal LINK1_M = 1.0;		// link mass
static const dReal LINK2_M = 1.0;		// link mass

// Some constants (sizes, radii, ...)
static const dReal SX = 0, SY = 0, SZ = 0.40;	// initial position of center of gravity
static const dReal l1 = 0.15, l2  = 0.17;  		// lenth of links
static const dReal sensor_l = 0.03;				// leg sensor length
static const dReal sensor_r = 0.03;				// leg sensor radius
static const dReal upset_s_l = 0.01;			// upset sensor length
static const dReal upset_s_r = 0.1;				// upset sensor radius 
static const dReal lx = 0.45, ly = 0.25, lz = 0.10;		// body sides
static const dReal r1 = 0.03, r2 = 0.03;				// leg radius
static const dReal cx1 = (lx-r1)/2, cy1 = (ly+r1)/2, cz1 = l1/2;     // temporal variable

// Centers of joints
static const dReal c_x[LEG_NUM][LINK_NUM] = {
	{ 0.9*cx1, 0.9*cx1},
	{	 -cx1,	  -cx1}, 
	{	 -cx1,	  -cx1},
	{ 0.9*cx1, 0.9*cx1}
};
static const dReal c_y[LEG_NUM][LINK_NUM] = {
	{ cy1,  cy1},
	{ cy1,  cy1},
	{-cy1, -cy1},
	{-cy1, -cy1}
};
static const dReal c_z[LEG_NUM][LINK_NUM] = {
	{0, -l1},
	{0, -l1},
	{0, -l1},
	{0, -l1}
};

// Links
static const dReal r[LINK_NUM] = {r1, r2};					// radius of links
static const dReal length[LINK_NUM] = {l1, l2};				// length of links
static const dReal weight[LINK_NUM] = {LINK1_M, LINK2_M};	// weight of links

// Link positions
static const dReal x[LEG_NUM][LINK_NUM] = {
	{ 0.9*cx1, 0.9*cx1},
	{    -cx1,    -cx1},
	{    -cx1,    -cx1},
	{ 0.9*cx1, 0.9*cx1}
};
static const dReal y[LEG_NUM][LINK_NUM] = {
	{ cy1,  cy1},
	{ cy1,  cy1},
	{-cy1, -cy1},
	{-cy1, -cy1}
};
static const dReal z[LEG_NUM][LINK_NUM] = {
	{-cz1, -cz1 - (l1+l2)/2},
	{-cz1, -cz1 - (l1+l2)/2},
	{-cz1, -cz1 - (l1+l2)/2},
	{-cz1, -cz1 - (l1+l2)/2}
};

// Axis of joint
static const dReal axis_x[LEG_NUM][LINK_NUM+1] = {{ 0, 0, 0}, { 0, 0, 0}, { 0, 0, 0}, { 0, 0, 0}};
static const dReal axis_y[LEG_NUM][LINK_NUM+1] = {{ 0, 1,-1}, { 0, 1,-1}, { 0, 1,-1}, { 0, 1,-1}};
static const dReal axis_z[LEG_NUM][LINK_NUM+1] = {{-1, 0, 0}, {-1, 0, 0}, { 1, 0, 0}, { 1, 0, 0}};



// Creates torso of the robot and adds it to the world
void Robot::createTorso() {
	dMass mass;
	torso_.body  = dBodyCreate(world_);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, TORSO_M, lx, ly, lz);
	dBodySetMass(torso_.body, &mass);	
	torso_.geom = dCreateBox(space_, lx, ly, lz);
	dGeomSetBody(torso_.geom, torso_.body);
	dBodySetPosition(torso_.body, SX, SY, SZ);
}

// Creates all 4 legs of the robot and adds them to the world
void Robot::createLegs() {
	int direction = 3;
	dMass mass;
	for (int i = 0; i < LEG_NUM; i++) {
		for (int j = 0; j < LINK_NUM; j++) {
			leg[i][j].body = dBodyCreate(world_);
			dBodySetPosition(leg[i][j].body, SX+x[i][j], SY+y[i][j], SZ+z[i][j]);
			dMassSetZero(&mass);
			dMassSetCapsuleTotal(&mass, weight[j], direction, r[j], length[j]);
			dBodySetMass(leg[i][j].body, &mass);
			leg[i][j].geom = dCreateCapsule(space_, r[j], length[j]);
			dGeomSetBody(leg[i][j].geom, leg[i][j].body);
		}
	}
}

// Creates leg sensors
void Robot::createLegSensors() {
	int direction = 3;
	dMass mass;
	for (int i = 0; i < LEG_NUM; i++) {
		sensor[i].body = dBodyCreate(world_);
		dMassSetZero(&mass);
		dMassSetCapsuleTotal(&mass, SENSOR_M, direction, sensor_r, sensor_l);
		dBodySetMass(sensor[i].body, &mass);
		dBodySetPosition(sensor[i].body, SX+x[i][0], SY+y[i][0], SZ-l1-l2-sensor_l/2);
		sensor[i].geom = dCreateCapsule(space_, sensor_r, sensor_l);
		dGeomSetBody(sensor[i].geom, sensor[i].body);
	}
}

// Creates upset sensors
void Robot::createUpsetSensors() {
	dMass mass;
	int direction = 3;

	// Create upset sensor(on back)
	upset_sensor[0].body = dBodyCreate(world_);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, SENSOR_M, direction, upset_s_r, upset_s_l);
	dBodySetMass(upset_sensor[0].body, &mass);
	dBodySetPosition(upset_sensor[0].body, SX, SY, SZ+lz/2);
	upset_sensor[0].geom = dCreateCylinder(space_, upset_s_r, upset_s_l);
	dGeomSetBody(upset_sensor[0].geom, upset_sensor[0].body);

	// Create upset sensor(on belly)
	upset_sensor[1].body = dBodyCreate(world_);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, SENSOR_M, direction, upset_s_r, upset_s_l);
	dBodySetMass(upset_sensor[1].body, &mass);
	dBodySetPosition(upset_sensor[1].body, SX, SY, SZ-lz/2);
	upset_sensor[1].geom = dCreateCylinder(space_, upset_s_r, upset_s_l);
	dGeomSetBody(upset_sensor[1].geom, upset_sensor[1].body);
}

// Creates shoulder joints (attaches legs to the torso)
void Robot::createShoulderJoints() {
	for (int i = 0; i < LEG_NUM; ++i) {	
		leg[i][0].joint = dJointCreateUniversal(world_, 0);
		dJointAttach(leg[i][0].joint, torso_.body, leg[i][0].body);

		dJointSetUniversalAnchor(leg[i][0].joint, SX+c_x[i][0], SY+c_y[i][0], SZ+c_z[i][0]);
		dJointSetUniversalAxis1(leg[i][0].joint, axis_x[i][0], axis_y[i][0], axis_z[i][0]);	//Axis1
		dJointSetUniversalAxis2(leg[i][0].joint, axis_x[i][1], axis_y[i][1], axis_z[i][1]);	//Axis2
		dJointSetUniversalParam(leg[i][0].joint, dParamLoStop1,  0);
		dJointSetUniversalParam(leg[i][0].joint, dParamHiStop1,  0.8*M_PI);
		dJointSetUniversalParam(leg[i][0].joint, dParamLoStop2, -0.5*M_PI);
		dJointSetUniversalParam(leg[i][0].joint, dParamHiStop2,  0.7*M_PI);
	}
}

// Creates knee joints (attaches 2 parts of the legs)
void Robot::createKneeJoints() {
	for (int i = 0; i < LEG_NUM; ++i) {
		leg[i][1].joint = dJointCreateHinge(world_, 0);
		dJointAttach(leg[i][1].joint, leg[i][0].body, leg[i][1].body);

		dJointSetHingeAnchor(leg[i][1].joint, SX+c_x[i][1], SY+c_y[i][1], SZ+c_z[i][1]);
		
		dJointSetHingeAxis(leg[i][1].joint, axis_x[i][2], axis_y[i][2], axis_z[i][2]);	//Axis
		dJointSetHingeParam(leg[i][1].joint, dParamLoStop, 0);
		dJointSetHingeParam(leg[i][1].joint, dParamHiStop, 0.8*M_PI);
	}
}

// Creates hoof feedback (attaches hoof sensors to legs)
void Robot::createHoofFeedback() {
	for (int i = 0; i < LEG_NUM; ++i){
		hoof[i] = dJointCreateFixed(world_, 0);
		dJointAttach(hoof[i], leg[i][1].body, sensor[i].body);
		dJointSetFixed(hoof[i]);
		dJointSetFeedback(hoof[i], &feedback[i]);
	}
}

// Creates upset sensor feedback (attaches sensors to the torso)
void Robot::createUpsetFeedback() {
	//upset sensor(on back)
	upset_fixed[0] = dJointCreateFixed(world_, 0);
	dJointAttach(upset_fixed[0], torso_.body, upset_sensor[0].body);
	dJointSetFixed(upset_fixed[0]);
	dJointSetFeedback(upset_fixed[0], &upset_feedback[0]);

	//upset sensor(on belly)
	upset_fixed[1] = dJointCreateFixed(world_, 0);
	dJointAttach(upset_fixed[1], torso_.body, upset_sensor[1].body);
	dJointSetFixed(upset_fixed[1]);
	dJointSetFeedback(upset_fixed[1], &upset_feedback[1]);
}


// Constructor, creates all body parts and sensors, attaches all body parts
// to each other and all sensors to the right places
Robot::Robot(dWorldID world, dSpaceID space, double one_step) 
	: world_(world), space_(space), one_step(one_step) {

	createTorso();
	createLegs();
	createLegSensors();
	createUpsetSensors();
	
	createShoulderJoints();
	createKneeJoints();

	createHoofFeedback();
	createUpsetFeedback();

	for (int i = 0; i < LEG_NUM; i++) {
		for (int j = 0; j < JT_NUM+1; j++) {
			//curr_state[i][j] = (rand() - RAND_MAX/2.) / (RAND_MAX/1.);
			curr_state[i][j] = 0.0;
		}
	}
}

// Destructor, destroy all joints of the robot
Robot::~Robot() {
	for (int i = 0; i < LEG_NUM; i++) {
		for (int j = 0; j < LINK_NUM; j++) {
			dJointDestroy(leg[i][j].joint);
			dBodyDestroy(leg[i][j].body);
			dGeomDestroy(leg[i][j].geom);
		}
		dJointDestroy(hoof[i]);
		dBodyDestroy(sensor[i].body);
		dGeomDestroy(sensor[i].geom);
	}

	dJointDestroy(upset_fixed[0]);
	dJointDestroy(upset_fixed[1]);
	dBodyDestroy(torso_.body);
	dBodyDestroy(upset_sensor[0].body);
	dBodyDestroy(upset_sensor[1].body);
	dGeomDestroy(torso_.geom);
	dGeomDestroy(upset_sensor[0].geom);
	dGeomDestroy(upset_sensor[1].geom);
}


// Control the robots legs
void Robot::PIDControl(double degree1, double degree2, double degree3, int legnum) {
	// PID constants
	static const dReal kp = 300.0 * one_step;
	static const dReal kd = 30.0 * one_step;
	static const dReal ki = 0.0 * one_step;
	static const dReal fMax = 4000.0 * one_step;
	
	dReal tmp, diff, u, omega;

	// Shoulder (back and forth)
	tmp = dJointGetUniversalAngle1(leg[legnum][0].joint);
	diff = degree1 - tmp;								
	diffsum[legnum][0] += diff;		
	omega = dJointGetUniversalAngle1Rate(leg[legnum][0].joint);	
	u = kp*diff + ki*diffsum[legnum][0] - kd*omega;		
	dJointSetUniversalParam(leg[legnum][0].joint,  dParamVel1, u);	
	dJointSetUniversalParam(leg[legnum][0].joint, dParamFMax1, fMax);	

	// Shoulder (up and down)
	tmp = dJointGetUniversalAngle2(leg[legnum][0].joint);	
	diff = degree2 - tmp;								
	diffsum[legnum][1] += diff;		
	omega = dJointGetUniversalAngle2Rate(leg[legnum][0].joint);	
	u = kp * diff + ki*diffsum[legnum][1] - kd * omega;								
	dJointSetUniversalParam(leg[legnum][0].joint,  dParamVel2, u);	
	dJointSetUniversalParam(leg[legnum][0].joint, dParamFMax2, fMax);	

	// Knee (up and down)
	tmp = dJointGetHingeAngle(leg[legnum][1].joint);	
	diff = degree3 - tmp;						
	diffsum[legnum][2] += diff;		
	omega = dJointGetHingeAngleRate(leg[legnum][1].joint);	
	u = kp * diff + ki*diffsum[legnum][2] - kd * omega;								
	dJointSetHingeParam(leg[legnum][1].joint,  dParamVel, u);	
	dJointSetHingeParam(leg[legnum][1].joint, dParamFMax, fMax);
}

// Walk the robot (move the robot)
void Robot::walk() {
	for (int i = 0; i < LEG_NUM; i++) {
		PIDControl(
			curr_state[i][0] * 0.8 * M_PI,
			(curr_state[i][1] * 1.2 - 0.5) * M_PI,
			curr_state[i][2] * 0.8 * M_PI,
			i
		);
	}
}

// Sets new state for the robot next move
void Robot::setNewState(const dReal (& new_state)[LEG_NUM][JT_NUM+1]) {
	for (int i = 0; i < LEG_NUM; i++) {
		for (int j = 0; j < JT_NUM; j++) {
			curr_state[i][j] = new_state[i][j];
		}
	}
}

// Getter for robots X position
double Robot::getXPosition() const {
	const double* pos = dBodyGetPosition(torso_.body);
	return pos[0];
}

// Returns value of back upset sensor
double Robot::getBackUpset() const {
	dJointFeedback *fb = dJointGetFeedback(upset_fixed[0]);
	return fb->f1[2];
}

// Returns value of front (belly) upset sensor
double Robot::getFrontUpset() const {
	dJointFeedback *fb = dJointGetFeedback(upset_fixed[1]);
	return fb->f1[2];
}

// Reads all sensor values into the 3 arrays
void Robot::readSensors(
	dReal (& hoof_force)[LEG_NUM],
	dReal (& angle)[LEG_NUM][JT_NUM+1],
	dReal (& upset_force)[2]
	) const {
	
	dJointFeedback *fb;

	// Read upset sensors
	fb = dJointGetFeedback(upset_fixed[0]);
	upset_force[0] = fb->f1[2];					// Z component
	fb = dJointGetFeedback(upset_fixed[1]);
	upset_force[1] = fb->f1[2];					// Z component

	// Read joint angles (shoulders and knees)
	for(int i = 0; i < LEG_NUM; ++i){
		angle[i][0] = dJointGetUniversalAngle1(leg[i][0].joint);	// shoulder (back and forward)
		angle[i][1] = dJointGetUniversalAngle2(leg[i][0].joint);	// shoulder (up and down)
		angle[i][2] = dJointGetHingeAngle(leg[i][1].joint);			// knee (up and down)
	}

	// Read hoof force sensors
	dReal fx, fy, fz;
	for(int i = 0; i < LEG_NUM; ++i){
		fb = dJointGetFeedback(hoof[i]);
		fx = fb->f1[0];
		fy = fb->f1[1];
		fz = fb->f1[2];
		hoof_force[i] = pow(fx*fx + fy*fy + fz*fz, 0.5);
	}
}


// Draws robot in the world ODE
void Robot::draw() const {
	dReal r, length;
	dVector3 sides;

	// Draw torso
	dsSetColor(TORSO_COLOR[0], TORSO_COLOR[1], TORSO_COLOR[2]);
	dGeomBoxGetLengths(torso_.geom, sides);
	dsDrawBox(dBodyGetPosition(torso_.body), dBodyGetRotation(torso_.body), sides);

	// Draw legs
	dsSetColor(LEG_COLOR[0], LEG_COLOR[1], LEG_COLOR[2]);
	for (int i = 0; i < LEG_NUM; i++) {
		for (int j = 0; j < LINK_NUM; j++ ) {
			dGeomCapsuleGetParams(leg[i][j].geom, &r,&length);
			dsDrawCapsule(
				dBodyGetPosition(leg[i][j].body),
				dBodyGetRotation(leg[i][j].body),
				length,
				r
			);
		}
	}

	// Draw leg sensors
	dsSetColor(SENSOR_COLOR[0], SENSOR_COLOR[1], SENSOR_COLOR[2]);
	for (int i = 0; i < LEG_NUM; i++) {
		dGeomCapsuleGetParams(sensor[i].geom, &r, &length);
		dsDrawCapsule(
			dBodyGetPosition(sensor[i].body),
			dBodyGetRotation(sensor[i].body),
			length,
			r
		);
	}

	// Draw upset sensor(on back)
	dGeomCylinderGetParams(upset_sensor[0].geom, &r, &length);
	dsDrawCylinder(
		dBodyGetPosition(upset_sensor[0].body),
		dBodyGetRotation(upset_sensor[0].body),
		length,
		r
	);

	// Draw upset sensor(on belly)
	dGeomCylinderGetParams(upset_sensor[1].geom, &r, &length);
	dsDrawCylinder(
		dBodyGetPosition(upset_sensor[1].body),
		dBodyGetRotation(upset_sensor[1].body),
		length,
		r
	);
}
