//PURPOSE : Class header file implementing the kinematics solution for the hand exoskeleton being developed in the ReNeu Robotics Lab at The University of Texas at Austin
//AUTHORS  : Priyanshu Agarwal
//CONTACT : mail2priyanshu@utexas.edu
//AFFILIATION : The University of Texas at Austin
//To DOs
//1. Add auto calibration functionality to automatically estimate the kinematic model parameters
//2.

/////////////////////////////////////////////////////////////////////
#ifndef DEFINITIONS_H
#define DEFINITIONS_H
#include "definitions.h"
#endif

class exo_finger {
    double x_A, y_A, r_j, l_AB, l_BC, l_CD, l_CE, l_EF, l_FG, l_GH, l_FI, l_IJ, l_JK, l_GK, l_AH; // user-defined coupled model parameters
    double t_GFK, t_HFG, t_DCH, l_KF, l_HF, l_CH, l_DH; // evaluated model parameters
    double t_abad, t_mcp, t_pip, t_dip; // human finger joint angles (absolute values with counterclockwise positive)
    double exo_abad, exo_t1, exo_t2, exo_x3, exo_t5, exo_t6, exo_t7, exo_t9, exo_t10, exo_x11; // finger exoskeleton angles/displacements
    double exo_t1_rel, exo_t5_rel, exo_t6_rel, exo_t9_rel; // relative finger exoskeleton angles measured by the sensors
    double t_mcp_rel, t_pip_rel, t_dip_rel; // human finger joint angles (relative values)
    Vector2d T_rel0, T_rel;

    // velocity variables
    double exo_t1_dot, exo_x3_dot, t_mcp_dot, exo_t5_dot, exo_t6_dot, t_pip_dot;
    double exo_t1_rel_dot, exo_t6_rel_dot;
    Vector4d X_dot;
    Vector2d T_dot, T_rel_dot; // T_rel_dot =[exo_t1_dot; exo_t6r_dot] = [-exo_t1_rel_dot; exo_t6_rel_dot]


    Matrix42d J, J_dot; // full Jacobian
    Matrix22d Jn, Jn_dot; // reduced space Jacobian
//    Matrix22d Jp; // parameter Jacobian for parameter adaptation

    // force/torque variables
//    Vector2d Tau_finger, Tau_finger_dot;
//    Vector2d Tau_exo, Tau_exo_dot;

    // variables for Jacobian calculation
    double t193, t196, t197, t198, t200, t201, t202, t203, t204, t205, t206, t207, t208, t209, t210, t211, t212;
    double t213, t214, t215, t216, t217, t218, t219, t220, t221, t222, t223, t224, t225;

    // variables for Jacobian derivative calculation
    double t36, t37, t38, t39, t40, t41, t42, t43, t44, t45, t46, t47, t48, t49;
    double t50, t51, t52, t53, t54, t55, t56, t57, t58, t59, t60, t61, t62, t63, t64, t65, t66, t67, t68, t69;
    double t70, t71, t72, t73, t74, t75, t76, t77, t78, t79, t80, t81, t82, t83, t84, t85, t86, t87, t88, t89;
    double t90, t91, t92, t93, t94, t95, t96, t97, t98, t99, t100, t101, t102, t103, t104, t105, t106, t107, t108, t109;
    double t110, t111, t112, t113, t114, t115, t116, t117, t118, t119;
    double t120, t121, t122, t123, t124, t125, t126, t127, t128, t129;
    double t130, t131, t132, t133, t134, t135, t136, t137, t138, t139;
    double t140, t141, t142, t143, t144, t145;

    // Finger Joint Torque Model Parameters
    // MCP Joint
    double A_mcp, B_mcp, C_mcp, D_mcp, E_mcp, F_mcp;
    // PIP Joint
    double A_pip, B_pip, C_pip, D_pip, E_pip, F_pip;

    Vector2d Tau_finger_min, Tau_finger_max;

    // SEA Parameters
    double rm, rj;
    Vector2d k;

    // Controller Parameters
    Vector2d Tau_exo_I;
    //PID gains
    Vector2d kp_exo, kd_exo, ki_exo; // exo torque control gains
    Vector2d kp_fin, kd_fin, ki_fin; // finger torque control gains
    Vector2d kp_fbl, kd_fbl, ki_fbl; // FBL gains

    // Estimator Parameters
    Vector3d P; // Parameter vector
    double Rp; // Measurement noise covariance matrix
    Matrix33d Sigma_p; // Parameter covariance matrix
    Vector3d Kp; // Kalman gain
    double dl_AH, dl_HF, dl_CD;
    Vector3d P_;
    Vector3d Hp;
    Vector2d exo_t5_val_l_AH, exo_t5_val_l_HF, exo_t5_val_l_CD;
    double exo_t5_rel_hat;

    // Error handling
    bool fail_flag;

  public:
    exo_finger(Vector2d &Theta_r, double l_AH0); // constructor to initialize model parameters and evaluate calculated model parameters
    bool exo_kinematics(double *exo_t, double *estimates, Vector2d T_dot); // closed-loop solution for exoskeleton kinematics
    bool exo_kin_eqns_mcp(double *x); // exoskeleton MCP chain kinematic solution
    bool exo_kin_eqns_pip(double *x); // exoskeleton PIP chain kinematic solution
    bool exo_kin_eqns_dip(double *x); // exoskeleton DIP chain kinematic solution
    void exo_calibration();
    void exo_jacobian();
    void exo_jacobian_dot();
    bool evaluate_joint_torque(Vector2d &Tau_finger, Vector2d &Tau_finger_dot);
    bool exo_inverse_statics(Vector2d &Tau_finger, Vector2d &Tau_finger_dot, Vector2d &Tau_exo, Vector2d &Tau_exo_dot);
    bool exo_forward_statics(Vector2d &Tau_finger, Vector2d &Tau_finger_dot, Vector2d &Tau_exo, Vector2d &Tau_exo_dot);
    bool exo_control_FF_exo(Vector2d &Tau_finger_d, Vector2d &Tau_exo_d, Vector2d &Theta_m, Vector2d &Tau_finger, Vector2d &Tau_exo, Vector2d &Theta_m_u);
    bool exo_control_FF_PID_exo(Vector2d &Tau_finger_d, Vector2d &Tau_finger_dot_d, Vector2d &Tau_exo_d, Vector2d &Tau_exo_dot_d, Vector2d &Theta_m, Vector2d &Theta_m_dot, Vector2d &Tau_finger, Vector2d &Tau_finger_dot, Vector2d &Tau_exo, Vector2d &Tau_exo_dot, Vector2d &Theta_m_u, Matrix22d &Jn_val, Matrix22d &Jn_dot_val);
    bool exo_control_FF_finger(Vector2d &Tau_finger_d, Vector2d &Tau_finger_dot_d, Vector2d &Tau_exo_d, Vector2d &Tau_exo_dot_d, Vector2d &Theta_m, Vector2d &Theta_m_dot, Vector2d &Tau_finger, Vector2d &Tau_finger_dot, Vector2d &Tau_exo, Vector2d &Tau_exo_dot, Vector2d &Theta_m_u);
    bool exo_control_FF_PID_finger(Vector2d &Tau_finger_d, Vector2d &Tau_finger_dot_d, Vector2d &Tau_exo_d, Vector2d &Tau_exo_dot_d, Vector2d &Theta_m, Vector2d &Theta_m_dot, Vector2d &Tau_finger, Vector2d &Tau_finger_dot, Vector2d &Tau_exo, Vector2d &Tau_exo_dot, Vector2d &Theta_m_u, Matrix22d &Jn, Matrix22d &Jn_dot);
    bool exo_control_FBL_finger(Vector2d &Tau_finger_d, Vector2d &Tau_finger_dot_d, Vector2d &Tau_finger_ddot_d, Vector2d &Tau_exo_d, Vector2d &Tau_exo_dot_d, Vector2d &Theta_m, Vector2d &Theta_m_dot, Vector2d &Tau_finger, Vector2d &Tau_finger_dot, Vector2d &Tau_exo, Vector2d &Tau_exo_dot, Vector2d &Theta_m_u, Matrix22d &Jn_val, Matrix22d &Jn_dot_val);
    bool exo_estimate_parameters();
    bool exo_kinematics_estimation();
    ~exo_finger(); // destructor
  };
