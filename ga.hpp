#ifndef GA_H_
#define GA_H_

#include <vector>
#include <random>


class GA {

private:
	static constexpr double MUTATE_DEV = 0.5;	// stddev for mutations

	std::default_random_engine generator;	// For random for mutation
	int* indices;							// For shuffling for selection

	double mutation_prob_;					// Probability of mutation
	int c_size_;							// Chromosome size
	int pop_size_;							// Population size
	int tournament_size_;					// Tournament size for tournament selection
	
	std::vector<double*> population_;		// Auxiliary array
	
private:
	int select(const std::vector<double>& fitness);
	void cross(double* parentA, double* parentB, double* childA, double* childB);
	void mutate(double* chrom);

public:
	GA(int c_size, int pop_size, double mutation_prob, int tournament_size);
	~GA();
	void evolve(const std::vector<double>& fitness, std::vector<double*>& population);

	static void randomizeChromosome(double* chromosome, int size);
	static void printChromosome(double* chromosome, int size);
};

#endif