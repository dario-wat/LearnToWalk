#ifndef GA_H_
#define GA_H_

#include <vector>


class GA {

private:
	double* fitness_;
	double mutation_prob_;
	int c_size_;
	int pop_size_;
	int tournament_size_;
	int* indices;
	std::vector<double*> population_;
	
private:
	int select(const std::vector<double>& fitness);
	void cross(double* parentA, double* parentB, double* childA, double* childB);
	void mutate(double* chrom);
public:
	GA(int c_size, int pop_size, double mutation_prob, int tournament_size);
	void evolve(const std::vector<double>& fitness, std::vector<double*>& population);

	static void randomizeChromosome(double* chromosome, int size);

	// TODO make destructor
};

#endif