// This is how I managed to compile this
//  g++ quadruped.cpp robot.cpp ga.cpp ann.cpp evaluator.cpp visualizator.cpp 
// -lode -lpthread -ldrawstuff -lX11 -lglut -lGL -lGLU -std=c++11 -DdDOUBLE

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

#include <mpi.h>


//TODO CFM and ERP parameters??

// ga.cpp
// TODO crossing maybe average

// ann.cpp
// TODO activation function

//evaluator
// TODO what fitness??
// TODO send erp and cfm params thrpough the constructor instead of using constants

// TODO in robot.cpp random initialization is maybe not so random
// TODO in robot.cpp walk function, there was some weird walking code

using std::cout;
using std::cin;
using std::cerr;
using std::endl;


// Constants for world initialization
static const double GRAVITY = -9.81;
static const double CFM_P = 10e-10;
static const double ERP_P = 0.2;
static const double para_K = 100000.0;	//elastic modulus
static const double para_C = 1000.0;	//viscous modulus

static const double SIM_TIME = 20.0;
static const double SIM_STEP = 0.005;
static const int GENERATIONS = 300;
static const int POP_SIZE = 100;

static const int in_l = LEG_NUM + LEG_NUM * (JT_NUM+1);
static const int mid_l = 12;
static const int out_l = LEG_NUM*(JT_NUM+1);
static const int c_size = (in_l+1)*mid_l + (mid_l+1)*out_l;

// Auxiliary arrays
static double f_layer[mid_l * (in_l+1)];
static double s_layer[out_l * (mid_l+1)];

// For simulation
static Evaluator* evaluator;
static Robot* rob;
static ANN* ann;
static GA* ga;
static std::vector<double*> population(POP_SIZE);
static std::vector<double> fitness(POP_SIZE);

dWorldID world;
dSpaceID space;

// MPI
int size;
int rank;


// Decodes chromosome into 2 weight matrices
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


// Runs learning process
static void simulate() {
	int L = POP_SIZE/P;
	R = N%P;
	I = (N+P-p-1)/P;

	int minIndex = rank * POP_SIZE / size;
	int maxIndex = (rank+1) * POP_SIZE / size;

	for (int j = 0; j < GENERATIONS; j++) {
		// MPI_Scatter(population, )
		if (rank == 0) {
			cerr << "Masta Process starts sending.\n";
			int receiver = 1;
			for (int receiver = 1; receiver<size;receiver++) {
				int start = receiver * POP_SIZE / size;
				int end = (receiver+1) * POP_SIZE / size;
				for (int i=start;i<end;i++) {
					MPI_Send(population[i], c_size, MPI_DOUBLE, receiver, 0, MPI_COMM_WORLD);
				}
			}
			cerr << "Masta Process is done sending.\n";
		} else {
			//cout << "rank " << rank << endl;
			for (int i=minIndex;i<maxIndex;i++) {
				MPI_Recv(population[i], c_size, MPI_DOUBLE, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
			}
		}

		for (int i = minIndex; i < maxIndex; i++) {
			//cout << "rank " << rank << endl;
			rob = new Robot(world, space, SIM_STEP);
			decode(f_layer, s_layer, population[i]);
			ann->setWeights(f_layer, s_layer);
			fitness[i] = evaluator->evaluate(rob, ann);
			delete rob;
			if (i == 0 || i == 1) {
				cerr << "Ind: " << i << " Fitness: " << fitness[i] << endl;	
			}
			
		}
		cerr << endl;

		// gather to master
		//MPI_Gather()
		if (rank != 0) {
			for (int i=minIndex;i<maxIndex;i++) {
				MPI_Send(population[i], c_size, MPI_DOUBLE, 0, 0, MPI_COMM_WORLD);
				MPI_Send(&fitness[i], 1, MPI_DOUBLE, 0, 0, MPI_COMM_WORLD);
			}
		} else {
			for (int sender=1;sender<size;sender++) {
				int start = sender * POP_SIZE / size;
				int end = (sender+1) * POP_SIZE / size;
				for (int i=start;i<end;i++) {
					MPI_Recv(population[i], c_size, MPI_DOUBLE, sender, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
					MPI_Recv(&fitness[i], 1, MPI_DOUBLE, sender, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
				}
			}
		}

		if (rank==0) {
			int best_idx = -1;
			double best_val = 0;
			for (int i = 0; i < POP_SIZE; i++) {
				if (fitness[i] > best_val) {
					best_val = fitness[i];
					best_idx = i;
				}
			}
			cerr << "Generation: " << j << " Score: " << fitness[best_idx] << endl;		
			cout << "Generation: " << j << " Score: " << fitness[best_idx] << endl;		
			//GA::printChromosome(population[best_idx], c_size);

			ga->evolve(fitness, population);
		}
	}
}


int main(int argc, char** argv) {
	/* Initialize MPI */

	MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &size);
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);

	srand(time(NULL));
	
	// Initialize things for ODE
	dInitODE2(0);
	world = dWorldCreate();        						// for dynamics
	space = dHashSpaceCreate(0);        				// for collision
	dGeomID ground = dCreatePlane(space, 0, 0, 1, 0);	// ground
	dWorldSetGravity(world, 0, 0, GRAVITY);
	dWorldSetCFM(world, 1.0/(SIM_STEP*para_K+para_C));
	dWorldSetERP(world, (SIM_STEP*para_K)/(SIM_STEP*para_K + para_C));
	
#ifndef DRAWIT
	// Random population
	for (int i = 0; i < POP_SIZE; i++) {
		population[i] = new double[c_size];
		GA::randomizeChromosome(population[i], c_size);
	}
	ga = new GA(c_size, POP_SIZE, 0.005, 3);
	ann = new ANN(in_l, mid_l, out_l);
	evaluator = new Evaluator(world, space, SIM_STEP, SIM_TIME);

	simulate();			// Learn

	for (int i = 0; i < POP_SIZE; i++) {
		delete population[i];
	}
	delete ga;
	delete ann;
	delete evaluator;
#endif

#ifdef DRAWIT
	double chr[c_size];
	for (int i = 0; i < c_size; i++) {
		scanf("%la", chr+i);
	}
	decode(f_layer, s_layer, chr);
	ann = new ANN(in_l, mid_l, out_l);
	ann->setWeights(f_layer, s_layer);
	rob = new Robot(world, space, SIM_STEP);
	Visualizator::simulationLoop(argc, argv, rob, ann, world, space, SIM_STEP);
	delete rob;
	delete ann;
#endif
	
	// Finalization and exiting
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();

	return 0;
}