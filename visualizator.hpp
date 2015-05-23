#ifndef VISUALIZATOR_H_
#define VISUALIZATOR_H_

#include <ode/ode.h>
#include "robot.hpp"
#include "ann.hpp"


class Visualizator {

private:
	static const int INPUT_SIZE = LEG_NUM + LEG_NUM * (JT_NUM+1);
	static const int CONTACT_ARR_SIZE = 20;
	static const int WINDOW_WIDTH = 800;
	static const int WINDOW_HEIGHT = 480;

	typedef struct {
		dWorldID world_;
		dJointGroupID contact_group_;
		dContact* contact;
		int size;
	} CallbackData;

	CallbackData data;

	Robot* robot_;
	ANN* ann_;
	dWorldID world_;
	dSpaceID space_;
	double sim_step_;

	dContact contact[CONTACT_ARR_SIZE];
	dReal input[INPUT_SIZE];
	dReal new_state[LEG_NUM][JT_NUM+1];

private:
	static void nearCallback(void *data, dGeomID o1, dGeomID o2);

public:
	// TODO make it read from a file or something
	Visualizator(Robot* robot, ANN* ann, dWorldID world, dSpaceID space, double sim_step);
	void simLoop();
};

#endif