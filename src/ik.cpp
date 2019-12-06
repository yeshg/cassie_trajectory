#include "mujoco.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

#include "ik.h"

#define __IK_TASK_SPACE_TOLERANCE 0.001
#define __IK_MIN_STEP_SIZE_NORM 0.0001
#define __IK_ACCEPTABLE_JOINT_VIOLATION 0.01
#define __IK_ACCEPTABLE_EQ_CON_VIOLATION 0.001


//Debug
// #define USEDEBUG

#ifdef USEDEBUG
#define DEBUG_MSG(str) do { std::cout << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif

bool cassie_ik(void* m_ptr, void* d_ptr, double lx, double ly, double lz,
               double rx, double ry, double rz,
               double comx, double comy, double comz,
               bool zero_hip_yaw)
{

    mjModel* m = static_cast<mjModel*> (m_ptr);
    mjData* d = static_cast<mjData*> (d_ptr);

    using namespace Eigen;

    d->qpos[0] = comx;
    d->qpos[1] = comy;
    d->qpos[2] = comz;

    

    // cassie mechanical model offset
    double offset_footJoint2midFoot = sqrt(pow((-0.052821 + 0.069746)/2, 2) + pow((0.092622 + 0.010224)/2, 2));

    mjtNum left_x[3] = {lx, ly, lz + 0.6 * offset_footJoint2midFoot};
    mjtNum right_x[3] = {rx, ry, rz + 0.6 * offset_footJoint2midFoot};


    int left_foot_id = mj_name2id(m, mjOBJ_BODY, "left-foot");
    int left_heel_spring_id = mj_name2id(m, mjOBJ_JOINT, "left-heel-spring");
    int left_shin_id = mj_name2id(m, mjOBJ_JOINT, "left-shin");
    int left_hip_yaw_id = mj_name2id(m, mjOBJ_JOINT, "left-hip-yaw");

    int right_foot_id = mj_name2id(m, mjOBJ_BODY, "right-foot");
    int right_heel_spring_id = mj_name2id(m, mjOBJ_JOINT, "right-heel-spring");
    int right_shin_id = mj_name2id(m, mjOBJ_JOINT, "right-shin");
    int right_hip_yaw_id = mj_name2id(m, mjOBJ_JOINT, "right-hip-yaw");

    DEBUG_MSG( "Ids for the joints in order: " 
                << left_foot_id << ", "
                <<  left_heel_spring_id << ", "
               << left_shin_id << ", "
               << left_hip_yaw_id << ", "
               << right_foot_id  << ", "
               << right_heel_spring_id  << ", "
               << right_shin_id  << ", "
               << right_hip_yaw_id );

    // int right_foot_id = mj_name2id(m, mjOBJ_BODY, "right-foot");
    // int pelvis_id = mj_name2id(m, mjOBJ_BODY, "cassie-pelvis");

    // Generalized coordinate column vector
    Map<Matrix<double, Dynamic, Dynamic, RowMajor>> q_pos(d->qpos, m->nq, 1);

    // End effector position Jacobian
    Matrix<double, Dynamic, Dynamic, RowMajor> J_p_left(3, m->nv);
    Matrix<double, Dynamic, Dynamic, RowMajor> J_p_right(3, m->nv);

    // End effector rotation Jacobian
    Matrix<double, Dynamic, Dynamic, RowMajor> J_r_left(3, m->nv);
    Matrix<double, Dynamic, Dynamic, RowMajor> J_r_right(3, m->nv);

    // Floating base Jacobian
    Matrix<double, Dynamic, Dynamic, RowMajor> J_f(3, m->nv);

    // Foot desired positions
    Map<Matrix<double, Dynamic, Dynamic, RowMajor>> left_x_des(left_x, 3, 1);
    Map<Matrix<double, Dynamic, Dynamic, RowMajor>> right_x_des(right_x, 3, 1);

    // Foot positions
    Map<Matrix<double, Dynamic, Dynamic, RowMajor>> left_x_pos(&d->xpos[3 * left_foot_id], 3, 1);
    Map<Matrix<double, Dynamic, Dynamic, RowMajor>> right_x_pos(&d->xpos[3 * right_foot_id], 3, 1);

    // Step direction
    Matrix<double, Dynamic, Dynamic, RowMajor> dq_l(m->nv, 1);
    Matrix<double, Dynamic, Dynamic, RowMajor> dq_r(m->nv, 1);

    Map<Matrix<double, Dynamic, Dynamic, RowMajor>> efc_J(d->efc_J, m->njmax, m->nv);

    //std::cout << efc_J << std::endl;
    // std::cout << G << std::endl;

    //The current task space error. Init to failure to hack the for loop to start
    double task_space_error = 2*__IK_TASK_SPACE_TOLERANCE;
    bool constraints_satisfied = false;
    double dq_step_norm = 2*__IK_MIN_STEP_SIZE_NORM;


    DEBUG_MSG("Entering IK Loop");

    // full body IK loop
    for (int iter_count = 0; 
        iter_count < 1000 &&  //Exit after 1000 iterations
        (task_space_error > __IK_TASK_SPACE_TOLERANCE || constraints_satisfied == false) && //Exit when constraints are satisifed and task space error is acceptable
        (dq_step_norm > __IK_MIN_STEP_SIZE_NORM || constraints_satisfied == false); //Exit when the step size is smaller than the minimum step size and constraints are satisfied 
        iter_count++)//10000 before, but simulation takes too long to render with that many steps
    {
        // prepare jacobians
        mj_kinematics(m, d);
        mj_comPos(m, d);
        mj_makeConstraint(m, d);

        DEBUG_MSG("mj state updated");

        // number of joint limit and equality constraints: total constraints - num friction - contacts
        int njl_eq = d->nefc - (d->nf + d->ncon);
        // number of equality constraints
        int n_eq = d->ne;

        // Full Constraint Jacobian
        Map<Matrix<double, Dynamic, Dynamic, RowMajor>> Jcon(d->efc_J, d->nefc, m->nv);

        // Equality constraint violation Jacobian
        MatrixXd G = MatrixXd::Zero(n_eq,m->nv);
        MatrixXd eq_con_violation = MatrixXd::Zero(n_eq,1);
        bool unacceptable_eq_con_violation= false;
        int currCons = 0;
        for(int i = 0; i < d->nefc; ++i){
            if(d->efc_type[i] == mjCNSTR_EQUALITY){
                G.block(currCons,0,1,m->nv) = Jcon.block(i,0,1,m->nv);
                eq_con_violation(currCons,0) = d->efc_pos[i];

                if( abs(eq_con_violation(currCons,0)) > __IK_ACCEPTABLE_EQ_CON_VIOLATION){
                    // std::cout << "Large EQ Constraint Violation\n";
                    unacceptable_eq_con_violation = true;
                }
                currCons++;
            }
        }


        DEBUG_MSG("Equality Constraint violation and jacobian calculated, Unacceptable:");
        DEBUG_MSG(unacceptable_eq_con_violation);

        // std::cout << eq_con_violation;

        // Joint Limit constraint violation Jacobian and violations
        MatrixXd Jlim = MatrixXd::Zero(njl_eq - n_eq,m->nv);
        MatrixXd lim_violation = MatrixXd::Zero(njl_eq - n_eq,1);
        bool unacceptable_joint_violation= false;
        currCons = 0;
        for(int i = 0; i < d->nefc; ++i){
            if(d->efc_type[i] == mjCNSTR_LIMIT_JOINT){
                Jlim.block(currCons,0,1,m->nv) = Jcon.block(i,0,1,m->nv);
                lim_violation(currCons,0) = d->efc_pos[i];

                if(lim_violation(currCons,0) < -__IK_ACCEPTABLE_JOINT_VIOLATION){
                    // std::cout << "Large Joint Violation\n";
                    unacceptable_joint_violation = true;
                }
                currCons++;
            }
        }

        DEBUG_MSG("Joint Limit constraint violation Jacobian and violations calculated, Unacceptable:");
        DEBUG_MSG(unacceptable_joint_violation);


        if(unacceptable_joint_violation == true){

            DEBUG_MSG("Entered Joint Violation Correction step");
            G.col(m->jnt_dofadr[left_heel_spring_id]).setZero();
            G.col(m->jnt_dofadr[left_shin_id]).setZero();
            G.col(m->jnt_dofadr[right_heel_spring_id]).setZero();
            G.col(m->jnt_dofadr[right_shin_id]).setZero();
            if(zero_hip_yaw == true){
                // G.col(m->jnt_dofadr[right_hip_yaw_id]).setZero();
                // G.col(m->jnt_dofadr[left_hip_yaw_id]).setZero();
            }

            MatrixXd Ginv = G.completeOrthogonalDecomposition().pseudoInverse();
            MatrixXd I = MatrixXd::Identity(Ginv.rows(), Ginv.rows());

            // Null space projector
            MatrixXd N = (I - G.transpose() * Ginv.transpose()).transpose();

            //Define joint limit correction step

            // std::cout << "G" << G.size();
            // std::cout << "lim_violation" << lim_violation.size();
            MatrixXd dq = 10 * -N * Jlim.transpose() * (lim_violation);

            

            mj_integratePos(m, q_pos.data(), dq.data(), 0.01);
        }
        else if(unacceptable_eq_con_violation == true){

            DEBUG_MSG("Entered Eq Constraint Correction step");
            
            G.col(m->jnt_dofadr[left_heel_spring_id]).setZero();
            G.col(m->jnt_dofadr[left_shin_id]).setZero();
            G.col(m->jnt_dofadr[right_heel_spring_id]).setZero();
            G.col(m->jnt_dofadr[right_shin_id]).setZero();
            if(zero_hip_yaw == true){
                // G.col(m->jnt_dofadr[right_hip_yaw_id]).setZero();
                // G.col(m->jnt_dofadr[left_hip_yaw_id]).setZero();
            }

            //Define joint limit correction step
            // std::cout << "G" << G.size();
            // std::cout << "lim_violation" << lim_violation.size();
            MatrixXd dq = -10 * G.transpose() * (eq_con_violation);

            mj_integratePos(m, q_pos.data(), dq.data(), 0.01);
        }
        else{

            DEBUG_MSG("Entered Task space grad step");
            // get end effector jacobian
            mj_jacBody(m, d, J_p_left.data(), J_r_left.data(), left_foot_id);
            mj_jacBody(m, d, J_p_right.data(), J_r_right.data(), right_foot_id);

            // Zero out jacobian columns relating to spring degrees of freedom
            // plus 2 because ball joint has 2 extra dof
            // G.col(left_heel_spring_id + 2).setZero();
            // G.col(left_shin_id + 2).setZero();

            G.col(m->jnt_dofadr[left_heel_spring_id]).setZero();
            G.col(m->jnt_dofadr[left_shin_id]).setZero();
            G.col(m->jnt_dofadr[right_heel_spring_id]).setZero();
            G.col(m->jnt_dofadr[right_shin_id]).setZero();
            if(zero_hip_yaw == true){
                // G.col(m->jnt_dofadr[right_hip_yaw_id]).setZero();
                // G.col(m->jnt_dofadr[left_hip_yaw_id]).setZero();
            }

            // J_p_left.col(left_heel_spring_id + 2).setZero();
            // J_p_left.col(left_shin_id + 2).setZero();

            J_p_left.col(m->jnt_dofadr[left_heel_spring_id]).setZero();
            J_p_left.col(m->jnt_dofadr[left_shin_id]).setZero();
            J_p_right.col(m->jnt_dofadr[right_heel_spring_id]).setZero();
            J_p_right.col(m->jnt_dofadr[right_shin_id]).setZero();

            // Zero out jacobian columns relating to left and right foot positions because of weird offset angle thing
            J_p_left.col(m->jnt_dofadr[left_foot_id]).setZero();
            J_p_right.col(m->jnt_dofadr[right_foot_id]).setZero();

            if(zero_hip_yaw == true){
                J_p_right.col(m->jnt_dofadr[right_hip_yaw_id]).setZero();
                J_p_left.col(m->jnt_dofadr[left_hip_yaw_id]).setZero();
            }

            // fix the pelvis
            for (int i = 0; i < 6; i++)
            {
                G.col(i).setZero();
                J_p_left.col(i).setZero();
                J_p_right.col(i).setZero();
            }

            MatrixXd Ginv = G.completeOrthogonalDecomposition().pseudoInverse();
            MatrixXd I = MatrixXd::Identity(Ginv.rows(), Ginv.rows());

            // Null space projector
            MatrixXd N = (I - G.transpose() * Ginv.transpose()).transpose();

            dq_l = 10 * -N * J_p_left.transpose() * (left_x_pos - left_x_des);
            dq_r = 10 * -N * J_p_right.transpose() * (right_x_pos - right_x_des);

            MatrixXd dq = dq_l + dq_r;

            //If there are active joint limits, zero out the projection of the step into the violations
            if( njl_eq - n_eq > 0){

                //Check if current step in moving into a joint limit
                for(int k = 0; k < njl_eq - n_eq; k++){
                    //This should be a scalar but use MatrixXd to avoid type errors
                    MatrixXd jl_piercing = Jcon.block(k,0,1,m->nv)*dq;
                    if( jl_piercing(0,0) < 0){
                        // std::cout << "Constraining Joint #" << k << std::endl;
                        dq = dq - Jcon.block(k,0,1,m->nv).transpose()*jl_piercing/Jcon.block(k,0,1,m->nv).squaredNorm();
                    }
                }
            }
            // std::cout << "rows: " << dq.rows() << "cols: " << dq.cols() << std::endl;


            // zero out floating base dof
            // dq.block(0, 0, 7, 1) = MatrixXd::Zero(7, 1);

            // zero out floating base linear dof
            dq.block(0, 0, 3, 1) = MatrixXd::Zero(3, 1);

            // velocity might have different dimension than position due to quaternions,
            // so we must integrate it
            mj_integratePos(m, q_pos.data(), dq.data(), 0.2);

            dq_step_norm = dq.norm();


            //mj_normalizeQuat(m, d->qpos);
        }


        //Check if both the joint violations and the equality constraints are satisfied
        constraints_satisfied = !unacceptable_joint_violation && !unacceptable_eq_con_violation;
        // Total linear error of the foot positions
        task_space_error = (right_x_pos - right_x_des).norm() + (right_x_pos - right_x_des).norm();

        DEBUG_MSG("iter: " << iter_count << "\tPass Constriant: " << constraints_satisfied << "\tTS err: " << task_space_error << "\t:dq_step_norm: " << dq_step_norm);

    }


    if( constraints_satisfied == true && task_space_error < __IK_TASK_SPACE_TOLERANCE){
        return true;
    }
    else{
        return false;
    }
    

    //std::cout << "actual ik foot pos (rx, ry, rz): (" << rx << ", " << ry << ", "  << rz << ")     (lx, ly, lz): (" << lx << ", " << ly << ", "  << lz << ")" << std::endl;

    // efc_J:
    // neq
    // nefc - neq

    // Map<Matrix<int, Dynamic, Dynamic, RowMajor>> efc_type(d->efc_type, 1, d->nefc);
    // std::cout << efc_type << std::endl;

    //std::cout << "NEFC: " << d->nefc << std::endl << efc_J << std::endl;

    // std::cout << left_x_pos.transpose() << " | " << left_x_des.transpose() << std::endl;
    // std::cin >> a;

}


//This function takes in body and foot linear velocity vectors and returns the joint space velocity
double* cassie_task_space_vel( void* m_ptr, void* d_ptr, double ldx, double ldy, double ldz,
               double rdx, double rdy, double rdz,
               double comdx, double comdy, double comdz)
{


    mjModel* m = static_cast<mjModel*> (m_ptr);
    mjData* d = static_cast<mjData*> (d_ptr);

    // prepare jacobians
    mj_kinematics(m, d);
    mj_comPos(m, d);
    mj_makeConstraint(m, d);

    using namespace Eigen;

    int left_foot_id = mj_name2id(m, mjOBJ_BODY, "left-foot");
    int left_heel_spring_id = mj_name2id(m, mjOBJ_JOINT, "left-heel-spring");
    int left_shin_id = mj_name2id(m, mjOBJ_JOINT, "left-shin");
    int left_hip_yaw_id = mj_name2id(m, mjOBJ_JOINT, "left-hip-yaw");

    int right_foot_id = mj_name2id(m, mjOBJ_BODY, "right-foot");
    int right_heel_spring_id = mj_name2id(m, mjOBJ_JOINT, "right-heel-spring");
    int right_shin_id = mj_name2id(m, mjOBJ_JOINT, "right-shin");
    int right_hip_yaw_id = mj_name2id(m, mjOBJ_JOINT, "right-hip-yaw");

    // End effector position Jacobian
    Matrix<double, Dynamic, Dynamic, RowMajor> J_p_left(3, m->nv);
    Matrix<double, Dynamic, Dynamic, RowMajor> J_p_right(3, m->nv);

    // End effector rotation Jacobian
    Matrix<double, Dynamic, Dynamic, RowMajor> J_r_left(3, m->nv);
    Matrix<double, Dynamic, Dynamic, RowMajor> J_r_right(3, m->nv);

    // get end effector jacobian
    mj_jacBody(m, d, J_p_left.data(), J_r_left.data(), left_foot_id);
    mj_jacBody(m, d, J_p_right.data(), J_r_right.data(), right_foot_id);

    //Zero out the spring joints of the end effector jacobians
    J_p_left.col(m->jnt_dofadr[left_heel_spring_id]).setZero();
    J_p_left.col(m->jnt_dofadr[left_shin_id]).setZero();
    J_p_right.col(m->jnt_dofadr[right_heel_spring_id]).setZero();
    J_p_right.col(m->jnt_dofadr[right_shin_id]).setZero();
    //Zero out the floating base linear and angular velocitities. first 6 elements
    J_p_left.block(0,0,6,m->nv).setZero();
    J_p_right.block(0,0,6,m->nv).setZero();

    // Generalized coordinate vel column vector
    double* q_vel_return = new double[m->nv];
    Map<Matrix<double, Dynamic, Dynamic, RowMajor>> q_vel(q_vel_return, m->nv, 1);
    Matrix<double, Dynamic, Dynamic, RowMajor> task_space_vel(6, 1);

    //No need for the jacobian here. Set floating base to match linear velocity
    //and rotational velocity to zero.
    q_vel(0,0) = comdx;
    q_vel(1,0) = comdy;
    q_vel(2,0) = comdz;
    q_vel.block(3,0,3,1).setZero();

    //Set desired task space velocities
    task_space_vel(0,0) = ldx;
    task_space_vel(1,0) = ldy;
    task_space_vel(2,0) = ldz;
    task_space_vel(3,0) = rdx;
    task_space_vel(4,0) = rdy;
    task_space_vel(5,0) = rdz;

    MatrixXd J_ts(6,m->nv-6);
    J_ts.block(0,0,3,m->nv-6) = J_p_left.block(0,5,3,m->nv-6);
    J_ts.block(0,3,3,m->nv-6) = J_p_right.block(0,5,3,m->nv-6);

    std::cout << "Velocity pre solve: \n" << q_vel << std::endl;

    q_vel.block(6,0,m->nv-6,1) = J_ts.colPivHouseholderQr().solve(task_space_vel);

    std::cout << "Velocity post solve: \n" << q_vel << std::endl;

    std::cout << "desired task space vel \n" << task_space_vel << std::endl;

    std::cout << "Resulting task space vel \n" << J_ts*q_vel << std::endl;

    return q_vel_return;
}