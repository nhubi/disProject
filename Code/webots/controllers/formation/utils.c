#include "utils.h"
#include "math.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// This file contains useful functions.                                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////


/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

/*
 * Keep given float number within interval {-limit, limit}
 */
void limitf(float *number, int limit) {
	if (*number > limit)
		*number = (float)limit;
	if (*number < -limit)
		*number = (float)-limit;
}

/*
 * Computes the norm of a vector of dimension dim. 
 */
float norm(float* vector, int dim) {
	float res = 0;
	int i;
	for(i = 0; i < dim; i++)
		res += vector[i]*vector[i];
	return sqrtf(res);
}

/*
 * Normalizes a vector of dimension dim. 
 * Output: normal_vec
 */
void normalize(float* normal_vec, float* vector, int dim) {
	int i;
	float vec_norm = norm(vector, dim);
	for(i = 0; i < dim; i++)
		normal_vec[i] = vector[i] / vec_norm;
}

/*
 * Computes the product of a vector of dimension dim by a factor "factor". 
 */
void multiply_vector_by(float* vector, int dim, float factor) {
	int i;
	for(i = 0; i < dim; i++) {
		vector[i] = vector[i] * factor;
	}
}

