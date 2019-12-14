// This simulation is heavily based off of the
// basic.cpp provided as an example in MuJoCO
// Originally written by Emo Todorov 2017
// Modified by Kevin Green 2018
// Modified by Yesh Godse 2019

#include "mujoco.h"
#include "glfw3.h"

#include "ik.h"
#include "mujSimulation.h"

#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

// BAD PRACTICE CHANGE THIS: using global to keep track of how many times renderWindow() is called
int renderCount = 0;

// Only using globals because the openGL callbacks do not work with anything
// but static methods, and I need to modify data relating to the simulation
// in the callback

std::mutex mtx;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

mjvCamera cam;  // abstract camera
mjvOption opt;  // visualization options
mjvScene scn;   // abstract scene
mjrContext con; // custom GPU context

mjModel *m_glob = NULL; // MuJoCo model
mjData *d_glob = NULL;  // MuJoCo data

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m_glob, d_glob);
        mj_forward(m_glob, d_glob);
    }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m_glob, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m_glob, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

mujSimulation::mujSimulation(bool render_sim)
{
    // check for mjkey.txt in ~/.mujoco and activate it
    char mjkey_path[4096 + 1024];

    char *home = getenv("HOME");
    if (home)
        snprintf(mjkey_path, sizeof mjkey_path, "%.4096s/.mujoco/mjkey.txt", home);

    mj_activate(mjkey_path);

    char error[1000] = "";
    // m = mj_loadXML("assets/cassie_leg_planar_fixed.xml", NULL, error, 1000);
    m = mj_loadXML("assets/cassie.xml", NULL, error, 1000);
    if (!m)
        mju_error_s("%.1000s", error);

    // make data
    d = mj_makeData(m);

    // initial state
    double standing_state_qpos[m->nq] = {0., 0., 1.01, 1., 0.,
                                         0., 0., 0.0045, 0., 0.4973,
                                         0.97848309, -0.01639972, 0.01786969, -0.20489646, -1.1997,
                                         0., 1.4267, 0., -1.5244, 1.5244,
                                         -1.5968, -0.0045, 0., 0.4973, 0.97861413,
                                         0.00386006, -0.01524022, -0.20510296, -1.1997, 0.,
                                         1.4267, 0., -1.5244, 1.5244, -1.5968};

    // set qpos of standing state
    for (int i = 0; i < m->nq; i++)
    {
        d->qpos[i] = standing_state_qpos[i];
    }

    render = render_sim;
    if(render)
    {
        // initialize OpenGL
        if (!glfwInit())
            mju_error("could not initialize GLFW");

        GLFWvidmode vmode = *glfwGetVideoMode(glfwGetPrimaryMonitor());

        window = glfwCreateWindow((2 * vmode.width) / 3, (2 * vmode.height) / 3, "IK", NULL, NULL);

        if (!window)
        {
            glfwTerminate();
            mju_error("could not create window");
        }

        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        // initialize visualization data structures
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);

        // create scene and context
        mjv_makeScene(m, &scn, 2000);
        mjr_makeContext(m, &con, mjFONTSCALE_150);

        // install GLFW mouse and keyboard callbacks
        glfwSetKeyCallback(window, keyboard);
        glfwSetCursorPosCallback(window, mouse_move);
        glfwSetMouseButtonCallback(window, mouse_button);
        glfwSetScrollCallback(window, scroll);
    }

    m_glob = m;
    d_glob = d;
}

bool mujSimulation::visualizeTrajectory(void)
{

    double trajectory[][9] = {{0.0, -0.15, 0.0, 0.0, 0.15, 0.0, 0.02, 0.0, 1.0},
                              {0.0, -0.15, 0.057, 0.0, 0.15, 0.0, 0.04, 0.0, 1.0},
                              {0.0, -0.15, 0.105, 0.0, 0.15, 0.0, 0.06, 0.0, 1.0},
                              {0.0, -0.15, 0.143, 0.0, 0.15, 0.0, 0.08, 0.0, 1.0},
                              {0.0, -0.15, 0.171, 0.0, 0.15, 0.0, 0.1, 0.0, 1.0},
                              {0.0, -0.15, 0.19, 0.0, 0.15, 0.0, 0.12, 0.0, 1.0},
                              {0.0, -0.15, 0.2, 0.0, 0.15, 0.0, 0.14, 0.0, 1.0},
                              {0.0, -0.15, 0.2, 0.0, 0.15, 0.0, 0.16, 0.0, 1.0},
                              {0.0, -0.15, 0.19, 0.0, 0.15, 0.0, 0.18, 0.0, 1.0},
                              {0.0, -0.15, 0.171, 0.0, 0.15, 0.0, 0.2, 0.0, 1.0},
                              {0.0, -0.15, 0.143, 0.0, 0.15, 0.0, 0.22, 0.0, 1.0},
                              {0.0, -0.15, 0.105, 0.0, 0.15, 0.0, 0.24, 0.0, 1.0},
                              {0.0, -0.15, 0.057, 0.0, 0.15, 0.0, 0.26, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.0, 0.28, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.0, 0.3, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.057, 0.32, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.105, 0.34, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.143, 0.36, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.171, 0.38, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.19, 0.4, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.2, 0.42, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.2, 0.44, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.19, 0.46, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.171, 0.48, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.143, 0.5, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.105, 0.52, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.057, 0.54, 0.0, 1.0},
                              {0.0, -0.15, 0.0, 0.0, 0.15, 0.0, 0.56, 0.0, 1.0}};

    int phase_len = 28;
    int wait_time = 100;
    while (true)
    {
        for (int i = 0; i < phase_len; i++)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));

            mtx.lock();

            // mj_step(m, d);

            // mjtNum simstart = d->time;
            // while( d->time - simstart < 1.0/60.0 )
            //     mj_step(m, d);

            // ik(0.2, 0, 0.1);

            bool posNotReached = true;

            // Edit: no longer bad hack
            // Note: bad trajectory hack
            cassie_ik(m, d, trajectory[i][0], trajectory[i][1], trajectory[i][2],
                      trajectory[i][3], trajectory[i][4], trajectory[i][5],
                      0.00 + 1 * trajectory[i][6], trajectory[i][7], trajectory[i][8], false);
            // cassie_ik(sin(i*))

            mtx.unlock();
        }
        // std::cout << "exited loop" << std::endl;
    }
}

bool mujSimulation::simulationStep(double *traj_pos, int wait_time){

    if(render)
    {
        if(glfwWindowShouldClose(window)){
            // close GLFW, free visualization storage
            glfwTerminate();
            mjv_freeScene(&scn);
            mjr_freeContext(&con);

            // free MuJoCo model and data, deactivate
            mj_deleteData(d);
            mj_deleteModel(m);
            mj_deactivate();
            return false;
        }


        // std::cout << "Starting first IK call: "  << std::endl; 
        bool success_lock = cassie_ik(m, d, traj_pos[0], traj_pos[1], traj_pos[2] + 0.02,
                                traj_pos[3], traj_pos[4], traj_pos[5] + 0.02,
                                traj_pos[6], traj_pos[7], traj_pos[8] + 0.02, true);

        if(success_lock == false){

            bool success_free = cassie_ik(m, d, traj_pos[0], traj_pos[1], traj_pos[2] + 0.02,
                                            traj_pos[3], traj_pos[4], traj_pos[5] + 0.02,
                                            traj_pos[6], traj_pos[7], traj_pos[8] + 0.02, false);
            std::cout << "Locked Hip Failed, Freed hip succeed: " << success_free << std::endl; 
            
            while(1){
                renderWindow();
            }
        }
        else
        {
            // std::cout << "Locked Hip Succeeded\n";
        }
        
        std::this_thread::sleep_for (std::chrono::milliseconds(66));
        
        renderWindow();
    }
    else
    {
        bool success_lock = cassie_ik(m, d, traj_pos[0], traj_pos[1], traj_pos[2] + 0.02,
                                traj_pos[3], traj_pos[4], traj_pos[5] + 0.02,
                                traj_pos[6], traj_pos[7], traj_pos[8] + 0.02, true);

        if(success_lock == false){

            bool success_free = cassie_ik(m, d, traj_pos[0], traj_pos[1], traj_pos[2] + 0.02,
                                            traj_pos[3], traj_pos[4], traj_pos[5] + 0.02,
                                            traj_pos[6], traj_pos[7], traj_pos[8] + 0.02, false);
        }
    }
}

double* mujSimulation::runTaskSpaceVel(double *traj_vel){
    return cassie_task_space_vel( m, d, traj_vel[0], traj_vel[1], traj_vel[2],
                            traj_vel[3], traj_vel[4], traj_vel[5],
                            traj_vel[6], traj_vel[7], traj_vel[8]);
}

void mujSimulation::renderWindow()
{
    renderCount++;

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

void mujSimulation::startSimulation()
{

    //std::thread simthread(&mujSimulation::visualizeTrajectory, this);

    while (!glfwWindowShouldClose(window))
    {
        // start exclusive access (block simulation thread)
        //mtx.lock();

        // handle events (calls all callbacks)
        glfwPollEvents();

        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

        // end exclusive access (allow simulation thread to run)
        //mtx.unlock();


        // render while simulation is running
        renderWindow();
    }

    //simthread.join();
}

extern "C"
{
    mujSimulation *mujSimulation_new(bool render) { return new mujSimulation(render); }
    double *fetch_cassie_ik(mujSimulation *sim, double traj_pos[], int steps)
    {

        int phase_len = 28;
        int wait_time = 100;
        // take some steps of simulation

        for(int i = 0; i < steps; i++)
        {
            sim->simulationStep(traj_pos, wait_time);
        }
        return sim->d->qpos;
    }


    double *fetch_cassie_ts_vels(mujSimulation *sim, double task_space_vel[]){
        return sim->runTaskSpaceVel(task_space_vel);
    }
}