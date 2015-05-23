#include "visualizator.hpp"
#include <iostream>

static const double CFM_P = 10e-10;
static const double ERP_P = 0.2;
static const double para_K = 100000.0;	//elastic modulus
static const double para_C = 1000.0;	//viscous modulus

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
dReal Visualizator::hoof_force[LEG_NUM];
dReal Visualizator::angle[LEG_NUM][JT_NUM+1];
dReal Visualizator::upset_force[2];


// Callback for collisions when things get near.
void Visualizator::nearCallback(void *data, dGeomID o1, dGeomID o2) {
	double sim_step_ = *((double*) data);
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) {
		return;
	}
	
	int n = dCollide(o1, o2, CONTACT_ARR_SIZE, &contact[0].geom, sizeof(dContact));
	for (int i = 0; i < n; i++) {
		contact[i].surface.mode = dContactSoftERP | dContactSoftCFM;
		contact[i].surface.mu = 100.0; 
		contact[i].surface.soft_erp = (sim_step_*para_K)/(sim_step_*para_K + para_C);		//ERP
		contact[i].surface.soft_cfm = 1.0/(sim_step_*para_K+para_C);		//CFM
		dJointID c = dJointCreateContact(world_, contact_group_, &contact[i]);
		dJointAttach(c, b1, b2);
	}
}

// Exit when q pressed
void Visualizator::command(int cmd) {
	if (cmd == 'q') {
		dsStop();
	}
}

// Visualize things
void Visualizator::simLoop(int pause) {
	if (!pause) {
		dSpaceCollide(space_, &sim_step_, nearCallback);
		dWorldStep(world_, sim_step_);
		
		robot_->readSensors(hoof_force, angle, upset_force);
		createInput();
		ann_->feedThrough(&input[0], &new_state[0][0]);
		robot_->setNewState(new_state);
		robot_->walk();
		robot_->draw();
		
		dJointGroupEmpty(contact_group_);
	}
}

// Clear joint group at the end
void Visualizator::stop() {
	dJointGroupDestroy(contact_group_);
}

// Helper function for creating input
void Visualizator::createInput() {
	for (int i = 0; i < LEG_NUM; i++) {
		input[i] = hoof_force[i];
	}
	for (int i = 0; i < LEG_NUM; i++) {
		for (int j = 0; j < JT_NUM+1; j++) {
			input[LEG_NUM + i*(JT_NUM+1) + j] = angle[i][j];
		}
	}
}

// Initializes data and start drawing process
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