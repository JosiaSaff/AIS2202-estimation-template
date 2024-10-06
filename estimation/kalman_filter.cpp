#include "estimation/kalman_filter.h"

using namespace estimation;


// Implement kalman filter turtorial


kalman_filter::kalman_filter(){

}


//The kalman filter addresses the general problem of trying to estimate the

// x_k = Ax_{k-1}+Bu_{k-1}+w_{k-1]
// z_k = Hx_k

//noicy measurement z = x + er
//with state matrix x = [[][]]

// time evolution:
// x_{k+1} = x_k

//we need to define how the system is evolving through time