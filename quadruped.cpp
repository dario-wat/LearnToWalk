// This is how I managed to compile this
//  g++ quadruped.cpp robot.cpp ga.cpp ann.cpp -lode -lpthread 
// -ldrawstuff -lX11 -lglut -lGL -lGLU -std=c++11 -DdDOUBLE

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "robot.hpp"
#include "ga.hpp"
#include "ann.hpp"
#include "evaluator.hpp"
#include "visualizator.hpp"


// ga.cpp
// TODO crossing maybe average


// TODO in robot.cpp random initialization is maybe not so random
// TODO in robot.cpp walk function, there was some weird walking code

using std::cout;
using std::cin;
using std::endl;


// Simulation window size


// Constants for world initialization
static const double GRAVITY = -9.81;
static const double CFM_P = 10e-10;
static const double ERP_P = 0.2;


static const double SIM_TIME = 20.0;
static const int GENERATIONS = 300;
static const int POP_SIZE = 100;

static const int in_l = LEG_NUM + LEG_NUM * (JT_NUM+1);
static const int mid_l = 10;
static const int out_l = LEG_NUM*(JT_NUM+1);
static const int c_size = (in_l+1)*mid_l + (mid_l+1)*out_l;


static double f_layer[mid_l * (in_l+1)];
static double s_layer[out_l * (mid_l+1)];


static Evaluator* evaluator;
static Robot* rob;
static ANN* ann;
static GA* ga;
static std::vector<double*> population(POP_SIZE);
static std::vector<double> fitness(POP_SIZE);


dWorldID world;
dSpaceID space;


static void encode(double (& fl)[mid_l * (in_l+1)],
	double (& sl)[out_l * (mid_l+1)],
	double* chromosome
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

static void decode(double (& fl)[mid_l * (in_l+1)],
	double (& sl)[out_l * (mid_l+1)],
	double* chromosome
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


void simulate() {
	for (int j = 0; j < GENERATIONS; j++) {
		for (int i = 0; i < POP_SIZE; i++) {
			rob = new Robot(world, space, 0.005);
			decode(f_layer, s_layer, population[i]);
			ann->setWeights(f_layer, s_layer);
			fitness[i] = evaluator->evaluate(rob, ann);
			delete rob;
			cout << "Ind: " << i << " Fitness: " << fitness[i] << endl;
		}
		cout << endl;
		ga->evolve(fitness, population);
		rob = new Robot(world, space, 0.005);
		decode(f_layer, s_layer, population[0]);
		ann->setWeights(f_layer, s_layer);
		cout << evaluator->evaluate(rob, ann) << endl;
		delete rob;
		GA::printChromosome(population[0], c_size);
		cout << j<< endl;
	}
}


void command(int cmd) {
	if (cmd == 'q') {
		exit(0);
	}
}

void initializeCallbacks(dsFunctions* fn) {
	fn->version = DS_VERSION;
	fn->start = 0;//&start;
	fn->step = 0;//&simLoop;
	fn->command = &command;
	fn->stop = 0;
	fn->path_to_textures = "textures";
}

int main(int argc, char** argv) {
	srand(time(NULL));
	
	// Initialize things for ODE
	dInitODE();
	world = dWorldCreate();        			// for dynamics
	space = dHashSpaceCreate(0);        		// for collision
	dGeomID ground = dCreatePlane(space, 0, 0, 1, 0);	// ground
	dsFunctions fn;           							// function of drawstuff
	initializeCallbacks(&fn);
	evaluator = new Evaluator(world, space, 0.005, SIM_TIME);
	
	// TODO he set gravitiy as 0 in the beginning
	dWorldSetGravity(world, 0, 0, GRAVITY);
	
	// TODO he did something else here
	dWorldSetCFM(world, CFM_P);
	dWorldSetERP(world, ERP_P);
	
	ann = new ANN(in_l, mid_l, out_l);

	for (int i = 0; i < POP_SIZE; i++) {
		population[i] = new double[c_size];
		GA::randomizeChromosome(population[i], c_size);
	}
	ga = new GA(c_size, POP_SIZE, 0.1, 15);

	// Run endless loop
	//dsSimulationLoop(argc, argv, WINDOW_WIDTH, WINDOW_HEIGHT, &fn);
	simulate();

	// double chr[c_size];
	// for (int i = 0; i < c_size; i++) {
	// 	scanf("%la", chr+i);
	// }
	// rob = new Robot(world, space, 0.005);
	// decode(f_layer, s_layer, chr);
	// ann->setWeights(f_layer, s_layer);
	// cout << evaluator->evaluate(rob, ann) << endl;

	// decode(f_layer, s_layer, chr);
	// GA::printChromosome(chr, c_size);
	// ann->setWeights(f_layer, s_layer);
	// Robot* r = new Robot(world, space, 0.005);
	// cout << "Eval " << evaluator->evaluate(r, ann);
	//Visualizator::simulationLoop(argc, argv, r, ann, world, space, 0.005);
	
	// Finalization and exiting
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();

	return 0;
}