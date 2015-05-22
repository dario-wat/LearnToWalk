#ifndef ROBOT_H_
#define ROBOT_H_

#include <ode/ode.h>


// Body part counts
static const int LEG_NUM = 4;			// leg count
static const int LINK_NUM = 2;			// link count on each leg
static const int JT_NUM = 2;			// joint count on each leg


typedef struct {
  dBodyID body;
  dGeomID geom;
  dJointID joint;
  dReal m, r, x, y, z;		// m:weight, r:radius, x,y,z:position
} MyLink;



class Robot {

private:
	MyLink torso_;
	MyLink leg[LEG_NUM][LINK_NUM];
	MyLink sensor[LEG_NUM];
	MyLink upset_sensor[2];

	dWorldID world_;
	dSpaceID space_;
	double one_step;

	dJointID hoof[LEG_NUM];
	dJointFeedback feedback[LEG_NUM];
	dJointFeedback upset_feedback[2];
	dJointID upset_fixed[2];

	dReal curr_state[LEG_NUM][JT_NUM+1];
	dReal diffsum[LEG_NUM][JT_NUM+1] = {	{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
											{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}	};

private:
	void createTorso();
	void createLegs();
	void createLegSensors();
	void createUpsetSensors();
	void createShoulderJoints();
	void createKneeJoints();
	void createHoofFeedback();
	void createUpsetFeedback();
	void PIDControl(double degree1, double degree2, double degree3, int legnum);

public:
	Robot(dWorldID world, dSpaceID space, double one_step);
	void walk();
	void setNewState(const dReal (& new_state)[LEG_NUM][JT_NUM+1]);
	double getXPosition() const;
	double getBackUpset() const;
	double getFrontUpset() const;
	void readSensors(
		dReal (& hoof_force)[LEG_NUM],
		dReal (& angle)[LEG_NUM][JT_NUM+1],
		dReal (& upset_force)[2]) const;
	void draw() const;

	~Robot();
};


#endif