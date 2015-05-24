#include "evaluator.hpp"
#include <ode/ode.h>
#include "robot.hpp"
#include <iostream>


static const double para_K = 100000.0;	//elastic modulus
static const double para_C = 1000.0;	//viscous modulus
static const double CFM_P = 10e-10;
static const double ERP_P = 0.2;
static const double UPSET_THRESHOLD = 10.0;


// Constructor, sets data for near callback function
Evaluator::Evaluator(dWorldID world, dSpaceID space, double sim_step, double sim_time) 
	: world_(world), space_(space), sim_step_(sim_step), sim_time_(sim_time),
	contact_group_(dJointGroupCreate(0)), time_since_start_(0), robot_(NULL), ann_(NULL) {

	data.world_ = world_;
	data.contact_group_ = contact_group_;
	data.contact = &contact[0];
	data.size = CONTACT_ARR_SIZE;
	data.sim_step_ = sim_step_;
}

// Destructor, destroys joint group
Evaluator::~Evaluator() {
	dJointGroupDestroy(contact_group_);
}

// Callback to detect collisions
// TODO change erp and cfm to constructor variables
void Evaluator::nearCallback(void *data, dGeomID o1, dGeomID o2) {
	CallbackData* dat = (CallbackData*) data;

	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) {
		return;
	}
	
	int n = dCollide(o1, o2, dat->size, &dat->contact[0].geom, sizeof(dContact));
	for (int i = 0; i < n; i++) {
		dat->contact[i].surface.mode = dContactSoftERP | dContactSoftCFM;
		dat->contact[i].surface.mu = 100.0; 
		dat->contact[i].surface.soft_erp = (dat->sim_step_*para_K)/(dat->sim_step_*para_K + para_C);		//ERP
		dat->contact[i].surface.soft_cfm = 1.0/(dat->sim_step_*para_K+para_C);		//CFM
		dJointID c = dJointCreateContact(dat->world_, dat->contact_group_, &dat->contact[i]);
		dJointAttach(c, b1, b2);
	}
}

// Helper function to flatten hoof_force and angle data into 1D array. Prepares it
// for ann input.
void Evaluator::createInput() {
	for (int i = 0; i < LEG_NUM; i++) {
		input[i] = hoof_force[i];
	}
	for (int i = 0; i < LEG_NUM; i++) {
		for (int j = 0; j < JT_NUM+1; j++) {
			input[LEG_NUM + i*(JT_NUM+1) + j] = angle[i][j];
		}
	}
}


// Evaluates the robot. This is basically the fitness function.
double Evaluator::evaluate(Robot* robot, ANN* ann) {
	this->robot_ = robot;
	this->ann_ = ann;
	time_since_start_ = 0;
	prev_pos_ = 0;

	double result = 0;
	while (time_since_start_ < sim_time_) {
		time_since_start_ += sim_step_;

		dSpaceCollide(space_, &data, &nearCallback);
		dWorldStep(world_, sim_step_);

		robot_->readSensors(hoof_force, angle, upset_force);
		createInput();
		ann_->feedThrough(&input[0], &new_state[0][0]);
		robot_->setNewState(new_state);
		robot_->walk();

		double curr_pos = robot_->getXPosition();
		if (curr_pos - prev_pos_ > 0.01) {
			result += 1;
			prev_pos_ = curr_pos;
		} else if (curr_pos - prev_pos_ < -0.01) {
			result -= 1;
			prev_pos_ = curr_pos;
		}

		dJointGroupEmpty(contact_group_);

		if (robot_->getFrontUpset() > UPSET_THRESHOLD || robot_->getBackUpset() > UPSET_THRESHOLD) {
			return 0;
		}
	}
	
	// double res = robot_->getXPosition();
	// return res < 0 ? 0 : res;
	return result < 0 ? 0 : result;
}