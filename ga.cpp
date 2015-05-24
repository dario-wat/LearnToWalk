#include "ga.hpp"
#include <algorithm>
#include <limits>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <random>
#include <cstdio>


// Constructor, creates indices array that is used to shuffle selection,
// creates auxiliary population array to copy the population and use it
// to create new one.
GA::GA(int c_size, int pop_size, double mutation_prob, int tournament_size) 
	: 	mutation_prob_(mutation_prob), c_size_(c_size),
		pop_size_(pop_size), tournament_size_(tournament_size), population_(pop_size) {

	// Initialize indices array for shuffling for selection
	this->indices = new int[pop_size];
	for (int i = 0; i < pop_size; i++) {
		indices[i] = i;
	}

	// Auxilliary array for population
	for (int i = 0; i < pop_size; i++) {
		population_[i] = new double[c_size];
	}
}

// Destructor, clean up auxiliary arrays
GA::~GA() {
	delete indices;
	for (int i = 0; i < pop_size_; i++) {
		delete population_[i];
	}
}

// Selects individuals based on tournament selection
int GA::select(const std::vector<double>& fitness) {
	std::random_shuffle(indices, indices + pop_size_);
	double best_value = std::numeric_limits<double>::lowest();
	int best_index = -1;
	for (int i = 0; i < tournament_size_; i++) {
		int idx = indices[i];
		if (fitness[idx] > best_value) {
			best_value = fitness[idx];
			best_index = idx;
		}
	}
	return best_index;
}

// Rand double [0..1]
inline static double randDouble() {
	return ((double) rand()) / RAND_MAX;
}

// Crosses 2 parents. Cuts the chromosomes in 2 and then assigns parts to children.
// Creates 2 children by crossing.
void GA::cross(double* parentA, double* parentB, double* childA, double* childB) {
#ifdef GACUT
	int cut = rand() % c_size_;
	for (int i = 0; i < cut; i++) {
		childA[i] = parentA[i];
		childB[i] = parentB[i];
	}
	for (int i = cut; i < c_size_; i++) {
		childA[i] = parentB[i];
		childB[i] = parentA[i];
	}
#endif

#ifndef GACUT
	for (int i = 0; i < c_size_; i++) {
		if (randDouble() < 0.5) {
			childA[i] = parentA[i];
			childB[i] = parentB[i];
		} else {
			childA[i] = parentB[i];
			childB[i] = parentA[i];
		}
	}	
#endif
}

// Mutates a chromosome. Mutates (mutation_prob_ * 100) % of genes. In each gene that
// is being mutated, the new double value is creates using normal distribution
// with old value set as mean
void GA::mutate(double* chrom) {
	for (int i = 0; i < c_size_; i++) {
		if (randDouble() < mutation_prob_) {
			std::normal_distribution<double> pdf(chrom[i], MUTATE_DEV);
			chrom[i] = pdf(generator);
		}
	}
}

// Copies array
static void copy(double* from, double* to, int size) {
	for (int i = 0; i < size; i++) {
		to[i] = from[i];
	}
}

// Evolves. Uses 2 best from the previous population in form of elitism, and then
// using selection, crossing and mutations create the rest of the new population.
// The new population is saved in population variable.
void GA::evolve(const std::vector<double>& fitness, std::vector<double*>& population) {
	// Make a copy of the population
	for (int i = 0; i < pop_size_; i++) {
		copy(population[i], population_[i], c_size_);
	}

	int start = 0;

#ifdef ELITISM
	// Elitism, take best 2
	double best_val = std::numeric_limits<double>::lowest();
	double s_best_val = std::numeric_limits<double>::lowest();
	int best_ind = -1, s_best_ind = -1;
	for (int i = 0; i < pop_size_; i++) {
		if (fitness[i] > best_val) {
			s_best_val = best_val;
			s_best_ind = best_ind;
			best_val = fitness[i];
			best_ind = i;
		} else if (fitness[i] > s_best_val) {
			s_best_val = fitness[i];
			s_best_ind = i;
		}
	}

	// Copy best 2 back in the population
	copy(population_[best_ind], population[0], c_size_);
	copy(population_[s_best_ind], population[1], c_size_);

	start = 2;
#endif

	// Create the rest
	for (int i = start; i < pop_size_; i+=2) {
		int j = select(fitness);
		int k = select(fitness);
		cross(population_[j], population_[k], population[i], population[i+1]);
		mutate(population[i]);
		mutate(population[i+1]);
	}
}

// Creates chromosome randomized. This is static function and is used to initialize
// all chromosomes.
void GA::randomizeChromosome(double* chromosome, int size) {
	for (int i = 0; i < size; i++) {
		chromosome[i] = (rand() - RAND_MAX/2.) / (RAND_MAX/8.0);
	}
}

// Used for debugging
void GA::printChromosome(double* chromosome, int size) {
	for (int i = 0; i < size; i++) {
		printf("%a ", chromosome[i]);		// Hex for precision
	}
	std::cout << std::endl;
}