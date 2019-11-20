#ifndef IK_H
#define IK_H

#include "mujoco.h"

void cassie_ik(void* m_ptr, void* d_ptr, double lx, double ly, double lz, double rx, double ry, double rz, double comx, double comy, double comz);

double* cassie_task_space_vel( void* m_ptr, void* d_ptr, double ldx, double ldy, double ldz,
               double rdx, double rdy, double rdz,
               double comdx, double comdy, double comdz);
// void ik(double x, double y, double z);

#endif // IK_H