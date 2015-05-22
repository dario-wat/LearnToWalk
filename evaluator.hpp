#ifndef EVALUATOR_H_
#define EVALUATOR_H_

#include <ode/ode.h>
#include "robot.hpp"
#include "ann.hpp"


class Evaluator {

private:
	static const int INPUT_SIZE = LEG_NUM + LEG_NUM * (JT_NUM+1);
	static const int CONTACT_ARR_SIZE = 20;

	static constexpr double UPSET_THRESHOLD = 10.0;
	static constexpr double CFM_P = 10e-10;
	static constexpr double ERP_P = 0.2;

	typedef struct {
		dWorldID world_;
		dJointGroupID contact_group_;
		dContact* contact;
		int size;
	} CallbackData;

	CallbackData data;

	const double sim_step_;
	const double sim_time_;
	const dWorldID world_;
	const dSpaceID space_;
	const bool draw_;
	
	// Auxiliary arrays
	dReal hoof_force[LEG_NUM];
	dReal angle[LEG_NUM][JT_NUM+1];
	dReal upset_force[2];
	dReal input[INPUT_SIZE];
	dReal new_state[LEG_NUM][JT_NUM+1];
	dContact contact[CONTACT_ARR_SIZE];

	dJointGroupID contact_group_;
	Robot* robot_;
	ANN* ann_;
	double time_since_start_;

private:
	static void nearCallback(void *data, dGeomID o1, dGeomID o2);
	void createInput();
public:
	Evaluator(dWorldID world, dSpaceID space, double sim_step, double sim_time, bool draw);
	double evaluate();
	void setParams(Robot* robot, ANN* ann);
};


#endif