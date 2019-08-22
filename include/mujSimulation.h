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

public:

    mujSimulation(void);

    bool visualizeTrajectory(void);

    bool simulationStep(double *trajectory);

    void renderWindow();

    void startSimulation();

private:

};

extern "C"
{
    mujSimulation *mujSimulation_new();
    double *fetch_cassie_ik(mujSimulation *sim, double traj_pos[], int steps, bool render);
}

#endif // MUJSIMULATION_H