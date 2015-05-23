#include "ann.hpp"
#include <cmath>

// Constructor, creates all arrays for weights and nodes. Adds bias term to
// input and hidden nodes.
ANN::ANN(int inp, int hid, int out) : inp_(inp), hid_(hid), out_(out) {
	this->fl_ = new double[hid * (inp+1)];
	this->sl_ = new double[out * (hid+1)];
	this->in_data_ = new double[inp+1];
	this->mid_data_ = new double[hid+1];
}

// Clear auxiliary arrays
ANN::~ANN() {
	delete fl_;
	delete sl_;
	delete in_data_;
	delete mid_data_;
}

// Copy arrays
static void copy(double* from, double* to, int size) {
	for (int i = 0; i < size; i++) {
		to[i] = from[i];
	}
}

// Set the weights of the ann
void ANN::setWeights(double *fl, double *sl) {
	copy(fl, fl_, hid_ * (inp_+1));
	copy(sl, sl_, out_ * (hid_+1));
}

// Matrix multiplication AB=C
static void multiply(double *A, double *B, double *C, int n, int m, int o) {
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < o; j++) {
			C[i*o+j] = 0;
			for (int k = 0; k < m; k++) {
				C[i*o+j] += A[i*m+k] * B[k*o+j];
			}
		}
	}
}

// Activation function, ReLU
static void activation(double *v, int n) {
	for (int i = 0; i < n; i++) {
		// if (v[i] < 0) {
		// 	v[i] /= 100;
		// }
		v[i] = tanh(v[i]);
	}
}

static void activationOut(double* v, int n) {
	for (int i = 0; i < n; i++) {
		v[i] = 1.0 / (1.0 + exp(-v[i]));
	}
}

// Feed ANN with input and receive output
void ANN::feedThrough(double* input, double* output) {
	copy(input, in_data_, inp_);
	in_data_[inp_] = 1;
	
	multiply(fl_, in_data_, mid_data_, hid_, inp_+1, 1);
	activation(mid_data_, hid_);
	mid_data_[hid_] = 1;

	multiply(sl_, mid_data_, output, out_, hid_+1, 1);
	activationOut(output, out_);
}