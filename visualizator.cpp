#include "visualizator.hpp"
#include <iostream>

static const double CFM_P = 10e-10;
static const double ERP_P = 0.2;

Robot* Visualizator::robot_;
ANN* Visualizator::ann_;
dWorldID Visualizator::world_;
dSpaceID Visualizator::space_;
double Visualizator::sim_step_;
dsFunctions Visualizator::fn_;
dJointGroupID Visualizator::contact_group_;

dContact Visualizator::contact[CONTACT_ARR_SIZE];
dReal Visualizator::input[INPUT_SIZE];
dReal Visualizator::new_state[LEG_NUM][JT_NUM+1];

	
void Visualizator::nearCallback(void *data, dGeomID o1, dGeomID o2) {
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) {
		return;
	}
	
	int n = dCollide(o1, o2, CONTACT_ARR_SIZE, &contact[0].geom, sizeof(dContact));
	for (int i = 0; i < n; i++) {
		contact[i].surface.mode = dContactSoftERP | dContactSoftCFM;
		contact[i].surface.mu = 100.0; 
		contact[i].surface.soft_erp = ERP_P;//(one_step*para_K)/(one_step*para_K + para_C);		//ERP
		contact[i].surface.soft_cfm = CFM_P;//1.0/(one_step*para_K+para_C);		//CFM
		dJointID c = dJointCreateContact(world_, contact_group_, &contact[i]);
		dJointAttach(c, b1, b2);
	}
}

void Visualizator::command(int cmd) {
	if (cmd == 'q') {
		dsStop();
	}
}

void Visualizator::simLoop(int pause) {
	dSpaceCollide(space_, 0, nearCallback);
	dWorldStep(world_, sim_step_);
	ann_->feedThrough(&input[0], &new_state[0][0]);
	robot_->setNewState(new_state);
	robot_->walk();
	robot_->draw();
	dJointGroupEmpty(contact_group_);
}

void Visualizator::stop() {
	// TODO do i enter here ever?
	std::cout << "just got here";
	dJointGroupDestroy(contact_group_);
}

void Visualizator::simulationLoop(int argc, char** argv, Robot* robot, ANN* ann,
	dWorldID world, dSpaceID space, double sim_step) {

	robot_ = robot;
	ann_ = ann;
	world_ = world;
	space_ = space;
	sim_step_ = sim_step;
	contact_group_ = dJointGroupCreate(0);

	fn_.version = DS_VERSION;
	fn_.start = 0;
	fn_.step = &Visualizator::simLoop;
	fn_.command = &command;
	fn_.stop = &stop;
	fn_.path_to_textures = "textures";
	
	dsSimulationLoop(argc, argv, WINDOW_WIDTH, WINDOW_HEIGHT, &fn_);
}