#ifndef MUJSIMULATION_H
#define MUJSIMULATION_H

#include "mujoco.h"
#include "glfw3.h"


class mujSimulation {
public:  //Leave all public for now so I don't have to write accessor methods

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data

GLFWwindow* window;
//_mjvGeom visGeoms[__MUJSIMULATION_MAX_VIS_GEOMS];
//int nVisGeoms = 0;

bool render = false;

public:

    mujSimulation(bool render_sim);

    bool visualizeTrajectory(void);

    bool simulationStep(double *traj_pos, int wait_time);

    void renderWindow();

    void startSimulation();

    double* runTaskSpaceVel(double *traj_vel);

private:

};

extern "C"
{
    mujSimulation *mujSimulation_new(bool render_sim);
    double *fetch_cassie_ik(mujSimulation *sim, double traj_pos[], int steps);
    double *fetch_cassie_ts_vels(mujSimulation *sim, double task_space_vel[]);
}

#endif // MUJSIMULATION_H