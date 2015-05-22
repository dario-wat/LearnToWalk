// This is how I managed to compile this
//  g++ quadruped.cpp robot.cpp ga.cpp ann.cpp -lode -lpthread 
// -ldrawstuff -lX11 -lglut -lGL -lGLU -std=c++11 -DdDOUBLE

#include <iostream>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "robot.hpp"
#include "ga.hpp"
#include "ann.hpp"



// Simulation window size
static const int WINDOW_WIDTH = 800;
static const int WINDOW_HEIGHT = 480;

// Constants for world initialization
static const double GRAVITY = -9.81;
static const double CFM_P = 10e-10;
static const double ERP_P = 0.2;

static const int CONTACT_ARR_SIZE = 20;

static const double SIM_TIME = 5.0;

static const int GENS = 300;
static const int POP_SIZE = 100;

double fitness[POP_SIZE];

int in_l = LEG_NUM + LEG_NUM + (JT_NUM+1);
	int mid_l = 10;
	int out_l = LEG_NUM*(JT_NUM+1);
int c_size = (in_l+1)*mid_l + (mid_l+1)*out_l;


// static const int LEG_NUM = 4;
// static const int LINK_NUM = 2;
// static const int JT_NUM = 2;

using std::cout;
using std::endl;

Robot* rob;
ANN* ann;
GA* ga;
std::vector<double*> population(POP_SIZE);


void start() {
}

dWorldID world;
dSpaceID space;
dJointGroupID contact_group;

static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) {
		return;
	}
	
	dContact contact[CONTACT_ARR_SIZE];

	int n = dCollide(o1, o2, CONTACT_ARR_SIZE, &contact[0].geom, sizeof(dContact));
	for (int i = 0; i < n; i++) {
		contact[i].surface.mode = dContactSoftERP | dContactSoftCFM;
		contact[i].surface.mu = 100.0; 
		contact[i].surface.soft_erp = ERP_P;//(one_step*para_K)/(one_step*para_K + para_C);		//ERP
		contact[i].surface.soft_cfm = CFM_P;//1.0/(one_step*para_K+para_C);		//CFM
		dJointID c = dJointCreateContact(world, contact_group, &contact[i]);
		dJointAttach(c, b1, b2);
	}
}
double time_since_start = 0.0;
int curr_idx = 0;


void createInput(
	dReal (& hoof_force)[LEG_NUM],
	dReal (& input)[LEG_NUM + LEG_NUM*(JT_NUM+1)],
	dReal (& angle)[LEG_NUM][JT_NUM+1]
	) {
	
	for (int i = 0; i < LEG_NUM; i++) {
		input[i] = hoof_force[i];
	}
	for (int i = 0; i < LEG_NUM; i++) {
		for (int j = 0; j < JT_NUM+1; j++) {
			input[LEG_NUM + i*(JT_NUM+1) + j] = angle[i][j];
		}
	}
}

void encode(
	double (& fl)[mid_l * (in_l+1)],
	double (& sl)[out_l * (mid_l+1)],
	double (& chromosome)[c_size]
	) {

	int f_lim = mid_l * (in_l+1);
	int s_lim = out_l * (mid_l+1);
	int k = 0;
	for (int i = 0; i < f_lim; i++) {
		chromosome[k] = fl[i];
		k++;
	}
	for (int i = 0; i < s_lim; i++) {
		chromosome[k] = sl[i];
		k++;
	}
}

void decode(double (& fl)[mid_l * (in_l+1)],
	double (& sl)[out_l * (mid_l+1)],
	double (& chromosome)[c_size]
	) {

	int f_lim = mid_l * (in_l+1);
	int s_lim = out_l * (mid_l+1);
	int k = 0;
	for (int i = 0; i < f_lim; i++) {
		fl[i] = chromosome[k];
		k++;
	}
	for (int i = 0; i < s_lim; i++) {
		sl[i] = chromosome[k];
		k++;
	}
}


void simLoop(int pause) {
	// TODO handle all contact_group
	dSpaceCollide(space, 0, &nearCallback);		// TODO middle argument is data
	dWorldStep(world, 0.005);
	time_since_start += 0.005;
	dReal hoof_force[LEG_NUM];
	dReal angle[LEG_NUM][JT_NUM+1];
	dReal upset_force[2];
	rob->readSensors(hoof_force, angle, upset_force);



	// TODO use ann to calculate new curr state for the robot
	dReal new_state[LEG_NUM][JT_NUM+1];
	dReal input[LEG_NUM + LEG_NUM*(JT_NUM+1)];
	
	createInput(hoof_force, input, angle);

	ann->feedThrough(&input[0], &new_state[0][0]);

	rob->setNewState(new_state);

	rob->walk();
	rob->draw();
	dJointGroupEmpty(contact_group);

	time_since_start += 0.005;

	if (time_since_start > SIM_TIME) {
		time_since_start = 0;
		cout << rob->getXPosition() << endl;
		delete rob;
		delete ann;
		rob = new Robot(world, space, 0.005);
		curr_idx++;
		//fitness[curr_idx] = ;

		if (curr_idx >= 100) {
			// TODO run GA

		}
	}
}

void command(int cmd) {
	if (cmd == 'q') {
		exit(0);
	}
}

void initializeCallbacks(dsFunctions* fn) {
	fn->version = DS_VERSION;
	fn->start = &start;
	fn->step = &simLoop;
	fn->command = &command;
	fn->stop = 0;
	fn->path_to_textures = "textures";
}




int main(int argc, char** argv) {
	
	// Initialize things for ODE
	dInitODE();
	world = dWorldCreate();        			// for dynamics
	space = dHashSpaceCreate(0);        		// for collision
	dGeomID ground = dCreatePlane(space, 0, 0, 1, 0);	// ground
	contact_group = dJointGroupCreate(0); // contact group for collision
	dsFunctions fn;           							// function of drawstuff
	initializeCallbacks(&fn);
	
	// TODO he set gravitiy as = in the beginning
	dWorldSetGravity(world, 0, 0, GRAVITY);
	
	// TODO he did something else here
	dWorldSetCFM(world, CFM_P);
	dWorldSetERP(world, ERP_P);

	rob = new Robot(world, space, 0.005);
	
	ann = new ANN(in_l, mid_l, out_l);


	for (int i = 0; i < POP_SIZE; i++) {
		population[i] = new double[c_size];
		GA::randomizeChromosome(population[i], c_size);
	}
	ga = new GA(c_size, POP_SIZE, 0.05, 10);

	// Run endless loop
	dsSimulationLoop(argc, argv, WINDOW_WIDTH, WINDOW_HEIGHT, &fn);

	// Finalization and exiting
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();

	return 0;
}