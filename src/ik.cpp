#include "mujoco.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

#include "ik.h"

#define __IK_ACCEPTABLE_JOINT_VIOLATION 0.01
#define __IK_ACCEPTABLE_EQ_CON_VIOLATION 0.001

void cassie_ik(void* m_ptr, void* d_ptr, double lx, double ly, double lz,
               double rx, double ry, double rz,
               double comx, double comy, double comz)
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

    int right_foot_id = mj_name2id(m, mjOBJ_BODY, "right-foot");
    int right_heel_spring_id = mj_name2id(m, mjOBJ_JOINT, "right-heel-spring");
    int right_shin_id = mj_name2id(m, mjOBJ_JOINT, "right-shin");

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

    // full body IK
    for (int iter_count = 0; iter_count < 1000; iter_count++)//10000 before, but simulation takes too long to render with that many steps
    {
        // prepare jacobians
        mj_kinematics(m, d);
        mj_comPos(m, d);
        mj_makeConstraint(m, d);

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

        std::cout << eq_con_violation;

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

        if(unacceptable_joint_violation == true){

            G.col(m->jnt_dofadr[left_heel_spring_id]).setZero();
            G.col(m->jnt_dofadr[left_shin_id]).setZero();
            G.col(m->jnt_dofadr[right_heel_spring_id]).setZero();
            G.col(m->jnt_dofadr[right_shin_id]).setZero();

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

            G.col(m->jnt_dofadr[left_heel_spring_id]).setZero();
            G.col(m->jnt_dofadr[left_shin_id]).setZero();
            G.col(m->jnt_dofadr[right_heel_spring_id]).setZero();
            G.col(m->jnt_dofadr[right_shin_id]).setZero();

            //Define joint limit correction step
            // std::cout << "G" << G.size();
            // std::cout << "lim_violation" << lim_violation.size();
            MatrixXd dq = -10 * G.transpose() * (eq_con_violation);

            mj_integratePos(m, q_pos.data(), dq.data(), 0.01);
        }
        else{

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

            // J_p_left.col(left_heel_spring_id + 2).setZero();
            // J_p_left.col(left_shin_id + 2).setZero();

            J_p_left.col(m->jnt_dofadr[left_heel_spring_id]).setZero();
            J_p_left.col(m->jnt_dofadr[left_shin_id]).setZero();
            J_p_right.col(m->jnt_dofadr[right_heel_spring_id]).setZero();
            J_p_right.col(m->jnt_dofadr[right_shin_id]).setZero();

            // Zero out jacobian columns relating to left and right foot positions because of weird offset angle thing
            J_p_left.col(m->jnt_dofadr[left_foot_id]).setZero();
            J_p_right.col(m->jnt_dofadr[right_foot_id]).setZero();

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
            if( njl_eq - n_eq != 0){
                for(int )
            }
            // std::cout << "rows: " << dq.rows() << "cols: " << dq.cols() << std::endl;

            // velocity might have different dimension than position due to quaternions,
            // so we must integrate it
            mj_integratePos(m, q_pos.data(), dq.data(), 0.01);

            // zero out floating base dof
            // dq.block(0, 0, 7, 1) = MatrixXd::Zero(7, 1);

            //mj_normalizeQuat(m, d->qpos);

            // print out quaternion
            // std::cout << dq.transpose() << std::endl;
            // std::cout << left_shin_id << " " << left_heel_spring_id << std::endl;
        }
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

// void ik(double x, double y, double z)
// {
//     int bodyid = mj_name2id(m, mjOBJ_BODY, "end0");

//     // MuJoCo matrix definitions
//     mjtNum point[3] = {x, y, z};

//     using namespace Eigen;

//     // End effector Jacobians
//     Matrix<double, Dynamic, Dynamic, RowMajor> J_p_left(3, m->nv);
//     Matrix<double, Dynamic, Dynamic, RowMajor> J_r_left(3, m->nv);

//     // Constraint Jacobian
//     Map<Matrix<double, Dynamic, Dynamic, RowMajor>> G(d->efc_J, m->neq * 3, m->nv);

//     Map<Matrix<double, Dynamic, Dynamic, RowMajor>> g_err(d->efc_J, 3, 1);

//     Map<Matrix<double, Dynamic, Dynamic, RowMajor>> x_des(point, 3, 1);

//     Map<Matrix<double, Dynamic, Dynamic, RowMajor>> x_pos(&d->xpos[3 * bodyid], 3, 1);
//     Map<Matrix<double, Dynamic, Dynamic, RowMajor>> q_pos(d->qpos, m->nv, 1);

//     Matrix<double, Dynamic, Dynamic, RowMajor> dq(1, m->nv);

//     Map<Matrix<double, Dynamic, Dynamic, RowMajor>> efc_J(d->efc_J, m->njmax, m->nv);

//     for (int i = 0; i < 100; i++)
//     {
//         // prepare jacobians
//         mj_kinematics(m, d);
//         mj_comPos(m, d);
//         mj_makeConstraint(m, d);

//         std::cout << m->neq << std::endl;
//         int a;
//         std::cin >> a;

//         MatrixXd Ginv = G.completeOrthogonalDecomposition().pseudoInverse();
//         MatrixXd I = MatrixXd::Identity(Ginv.rows(), Ginv.rows());

//         MatrixXd N = (I - G.transpose() * Ginv.transpose()).transpose();

//         // get jacobian
//         mj_jacBody(m, d, J_p_left.data(), J_r_left.data(), bodyid);

//         dq = N * J_p_left.transpose() * (x_pos - x_des);

//         q_pos -= dq;
//     }
// }