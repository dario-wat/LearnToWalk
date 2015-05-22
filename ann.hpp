#ifndef ANN_H_
#define ANN_H_

class ANN {

private:
	int inp_;
	int hid_;
	int out_;
	double* fl_;				// First layer of weights
	double* sl_;				// Second layer of weights
	double* in_data_;			// Input data (aux array)
	double* mid_data_;			// Mid result data (aux array)

public:
	ANN(int inp, int hid, int out);
	void setWeights(double *fl, double *sl);
	void feedThrough(double* input, double* output);

	// TODO make destructor
};

#endif