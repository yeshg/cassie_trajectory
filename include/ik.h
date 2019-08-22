#ifndef IK_H
#define IK_H

#include "mujoco.h"

void cassie_ik(mjModel* m, mjData* d, double lx, double ly, double lz, double rx, double ry, double rz, double comx, double comy, double comz);

// void ik(double x, double y, double z);

#endif // IK_H