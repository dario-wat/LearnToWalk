#ifndef EVALUATOR_H_
#define EVALUATOR_H_

#include <ode/ode.h>
#include "robot.hpp"
#include "ann.hpp"


class Evaluator {

private:
	static const int INPUT_SIZE = LEG_NUM + LEG_NUM * (JT_NUM+1);
	static const int CONTACT_ARR_SIZE = 20;

	// Structure for sending the data into callback function
	typedef struct {
		dWorldID world_;
		dJointGroupID contact_group_;
		dContact* contact;
		int size;
		double sim_step_;
	} CallbackData;

	CallbackData data;				// Variable for that data

	const double sim_step_;			// One step in simulation
	const double sim_time_;			// How long to simulate
	const dWorldID world_;			// World to use
	const dSpaceID space_;			// Space to use
	
	// Auxiliary arrays
	dReal hoof_force[LEG_NUM];
	dReal angle[LEG_NUM][JT_NUM+1];
	dReal upset_force[2];
	dReal input[INPUT_SIZE];
	dReal new_state[LEG_NUM][JT_NUM+1];
	dContact contact[CONTACT_ARR_SIZE];

	dJointGroupID contact_group_;		// Joint group for collisions
	Robot* robot_;						// Robot to evaluate
	ANN* ann_;							// Ann for that robot
	double time_since_start_;			// Time counter

private:
	static void nearCallback(void *data, dGeomID o1, dGeomID o2);
	void createInput();
public:
	Evaluator(dWorldID world, dSpaceID space, double sim_step, double sim_time);
	~Evaluator();
	double evaluate(Robot* robot, ANN* ann);
};


#endif