#include "ga.hpp"
#include <algorithm>
#include <limits>
#include <ctime>
#include <cstdlib>


// TODO do sometihng with chromosome type, cant go around like this
// TODO change all array parameters


GA::GA(int c_size, int pop_size, double mutation_prob, int tournament_size) 
	: 	mutation_prob_(mutation_prob), c_size_(c_size),
		pop_size_(pop_size), tournament_size_(tournament_size), population_(pop_size) {

	// Randomizer
	srand(time(NULL));

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

void GA::cross(double* parentA, double* parentB, double* childA, double* childB) {
	int cut = rand() % c_size_;
	for (int i = 0; i < cut; i++) {
		childA[i] = parentA[i];
		childB[i] = parentB[i];
	}
	for (int i = cut; i < c_size_; i++) {
		childA[i] = parentB[i];
		childB[i] = parentA[i];
	}
	
	// TODO crossing maybe average
}

inline static double randDouble() {
	return ((double) rand()) / RAND_MAX;
}

inline static void copy(double* from, double* to, int size) {
	for (int i = 0; i < size; i++) {
		to[i] = from[i];
	}
}

void GA::mutate(double* chrom) {
	for (int i = 0; i < c_size_; i++) {
		if (randDouble() < mutation_prob_) {
			// TODO do some kind of mutation to chrom
		}
	}
}

void GA::evolve(const std::vector<double>& fitness, std::vector<double*>& population) {
	// Make a copy of the population
	for (int i = 0; i < pop_size_; i++) {
		copy(population[i], population_[i], c_size_);
	}

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

	for (int i = 2; i < pop_size_; i++) {
		int j = select(fitness);
		int k = select(fitness);
		cross(population_[j], population_[k], population[i], population[i+1]);
		mutate(population[i]);
		mutate(population[i+1]);
	}
}

void GA::randomizeChromosome(double* chromosome, int size) {
	for (int i = 0; i < size; i++) {
		chromosome[i] = randDouble() - 0.5;
	}
}