#ifndef UTILS_H
#define UTILS_H

void limit(int *number, int limit);
void limitf(float *number, int limit);
float norm(float* vector, int dim);
void normalize(float* normal_vec, float* vector, int dim);
void multiply_vector_by(float* vector, int dim, float factor);
void difference(float* vector1,float* vector2,float* difference,int dim);

#endif
