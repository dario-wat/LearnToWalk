#ifndef VISUALIZATOR_H_
#define VISUALIZATOR_H_

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "robot.hpp"
#include "ann.hpp"


class Visualizator {

private:
	static const int INPUT_SIZE = LEG_NUM + LEG_NUM * (JT_NUM+1);
	static const int CONTACT_ARR_SIZE = 20;
	static const int WINDOW_WIDTH = 800;
	static const int WINDOW_HEIGHT = 480;

	static Robot* robot_;
	static ANN* ann_;
	static dWorldID world_;
	static dSpaceID space_;
	static double sim_step_;
	static dsFunctions fn_;
	static dJointGroupID contact_group_;

	static dContact contact[CONTACT_ARR_SIZE];
	static dReal input[INPUT_SIZE];
	static dReal new_state[LEG_NUM][JT_NUM+1];

private:
	static void nearCallback(void *data, dGeomID o1, dGeomID o2);
	static void simLoop(int pause);
	static void command(int cmd);
	static void stop();

public:
	static void simulationLoop(int argc, char** argv, Robot* robot, ANN* ann, dWorldID world,
		dSpaceID space, double sim_step);

};

#endif