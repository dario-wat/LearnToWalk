#ifndef ANN_H_
#define ANN_H_


class ANN {

private:
	int inp_;			// Number of input nodes
	int hid_;			// Number of hidden nodes
	int out_;			// Number of output nodes

	// Arrays for weights
	double* fl_;				// First layer of weights
	double* sl_;				// Second layer of weights
	
	// Auxiliary arrays for nodes
	double* in_data_;			// Input data (aux array)
	double* mid_data_;			// Mid result data (aux array)

public:
	ANN(int inp, int hid, int out);
	~ANN();
	void setWeights(double *fl, double *sl);
	void feedThrough(double* input, double* output);
};

#endif