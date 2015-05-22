#include "ann.hpp"


ANN::ANN(int inp, int hid, int out) : inp_(inp), hid_(hid), out_(out) {
	int fl_size = hid * (inp+1);
	int sl_size = out * (hid+1);
	this->fl_ = new double[fl_size];
	this->sl_ = new double[sl_size];
	this->in_data_ = new double[inp+1];
	this->mid_data_ = new double[hid+1];
}

inline static void copy(double* from, double* to, int size) {
	for (int i = 0; i < size; i++) {
		to[i] = from[i];
	}
}

void ANN::setWeights(double *fl, double *sl) {
	copy(fl, fl_, inp_ * hid_);
	copy(sl, sl_, hid_ * out_);
}


inline static void multiply(double *A, double *B, double *C, int n, int m, int o) {
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < o; j++) {
			C[i*o+j] = 0;
			for (int k = 0; k < m; k++) {
				C[i*o+j] += A[i*m+k] * B[k*o+j];
			}
		}
	}
}

inline static void activation(double *v, int n) {
	for (int i = 0; i < n; i++) {
		if (v[i] < 0) {
			v[i] /= 100;
		}
	}
}

void ANN::feedThrough(double* input, double* output) {
	copy(input, in_data_, inp_);
	in_data_[inp_] = 1;
	
	multiply(fl_, in_data_, mid_data_, hid_, inp_+1, 1);
	activation(mid_data_, hid_);
	mid_data_[hid_] = 1;

	multiply(sl_, mid_data_, output, out_, hid_+1, 1);
	activation(output, out_);
}