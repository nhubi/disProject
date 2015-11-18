#include "utils.h"

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
