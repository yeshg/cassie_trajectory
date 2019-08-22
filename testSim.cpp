#include "mujSimulation.h"
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, const char** argv){


	// mujSimulation sim = mujSimulation();

    // // bool exit = false;

    double traj_pos[] = {0.0, -0.15, 0.0, 0.0, 0.15, 0.171, 0.38, 0.0, 1.0};
    // // int phase_len = 28;
    // // int wait_time = 100;
    // // for(int i = 0; i < 100; i++)
    // // {
    // //     //std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
    // //     sim.simulationStep(traj_pos);
    // //     sim.renderWindow();
    // // }

    // // std::cout << "FOOOOO" << std::endl;

    mujSimulation* sim = mujSimulation_new();

    std::cout << *fetch_cassie_ik(sim, traj_pos, 50, true) << std::endl; 

    return 0;

}