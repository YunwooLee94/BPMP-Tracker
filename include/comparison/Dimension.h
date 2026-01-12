// Dimension.h
#ifndef BPMP_TRACKER_DIMENSION_H
#define BPMP_TRACKER_DIMENSION_H

// Horizon length
const int N  = 10;

// Baseline에서 쓰는 “결정론적” tracker state dimension (x,y,theta)
// const int Nx = 3;

// control dimension (python: u=[omega, acc])
const int Nu = 2;

// legacy / placeholder
// const int Nc = 10;

// Belief dimensions used inside SCP cost evaluation
// robot belief: [x,y,theta], target belief: [x,y]
const int Nx_r = 3;
const int Nx_t = 2;

#endif
