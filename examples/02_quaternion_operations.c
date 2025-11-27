/*!=============================================================================
  ==============================================================================

  \file    02_quaternion_operations.c

  \author  SL Examples
  \date    2024

  ==============================================================================
  \remarks

  Example demonstrating quaternion operations available in the SL library.
  
  This example shows how to use:
  - Euler angles to quaternion conversion (eulerToQuat, eulerToQuatInv)
  - Quaternion to Euler angle conversion (quatToEuler, quatToEulerInv)
  - Quaternion to rotation matrix (quatToRotMat, quatToRotMatInv)
  - Quaternion derivatives (quatDerivatives)
  - Quaternion error computation (quatError, quatErrorVector)

  Compile with:
    gcc -I$LAB_ROOT/include -I../include -L$LAB_LIBDIR -o 02_quaternion_operations \
        02_quaternion_operations.c -lSLcommon -lutility -lm

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// SL specific headers
#include "SL.h"
#include "SL_common.h"
#include "utility.h"

/* Define PI if not already defined */
#ifndef PI
#define PI 3.14159265358979323846
#endif

/**
 * \brief Print a quaternion
 * 
 * \param[in] q       The quaternion to print
 * \param[in] label   Label for the quaternion
 */
void print_quat(SL_quat *q, const char *label) {
    printf("%s: [w=%.6f, x=%.6f, y=%.6f, z=%.6f]\n",
           label, q->q[_Q0_], q->q[_Q1_], q->q[_Q2_], q->q[_Q3_]);
}

/**
 * \brief Print Euler angles from a Vector
 * 
 * \param[in] angles  Vector of Euler angles [alpha, beta, gamma]
 * \param[in] label   Label for the angles
 */
void print_euler(Vector angles, const char *label) {
    printf("%s: [alpha=%.6f, beta=%.6f, gamma=%.6f] rad\n",
           label, angles[_A_], angles[_B_], angles[_G_]);
    printf("         [alpha=%.2f, beta=%.2f, gamma=%.2f] deg\n",
           angles[_A_] * 180.0 / PI,
           angles[_B_] * 180.0 / PI,
           angles[_G_] * 180.0 / PI);
}

/**
 * \brief Print a 3x3 rotation matrix
 * 
 * \param[in] R       The rotation matrix
 * \param[in] label   Label for the matrix
 */
void print_rotation_matrix(Matrix R, const char *label) {
    printf("%s:\n", label);
    printf("  [%8.5f  %8.5f  %8.5f]\n", R[1][1], R[1][2], R[1][3]);
    printf("  [%8.5f  %8.5f  %8.5f]\n", R[2][1], R[2][2], R[2][3]);
    printf("  [%8.5f  %8.5f  %8.5f]\n", R[3][1], R[3][2], R[3][3]);
}

/**
 * \brief Compute the norm of a quaternion
 * 
 * \param[in] q    The quaternion
 * \return         The norm of the quaternion
 */
double quat_norm(SL_quat *q) {
    return sqrt(q->q[_Q0_] * q->q[_Q0_] + 
                q->q[_Q1_] * q->q[_Q1_] + 
                q->q[_Q2_] * q->q[_Q2_] + 
                q->q[_Q3_] * q->q[_Q3_]);
}

/**
 * \brief Main function demonstrating quaternion operations
 * 
 * This example shows how to:
 * 1. Convert between Euler angles and quaternions
 * 2. Convert quaternions to rotation matrices
 * 3. Compute quaternion derivatives from angular velocity
 * 4. Compute error between quaternions
 */
int main(int argc, char **argv) {
    
    printf("=================================================\n");
    printf("SL Library Example: Quaternion Operations\n");
    printf("=================================================\n");
    
    /* Allocate rotation matrices and euler angle vectors using utility library */
    Matrix R;
    Vector euler;
    Vector euler_back;
    
    R = my_matrix(1, 3, 1, 3);
    euler = my_vector(1, 3);
    euler_back = my_vector(1, 3);
    
    /* ----------------------------------------------------------------
     * Example 1: Euler Angles to Quaternion Conversion
     * ----------------------------------------------------------------
     * SL uses the convention:
     * - eulerToQuat: Converts Euler angles to quaternion for global->local transform
     * - eulerToQuatInv: Converts Euler angles to quaternion for local->global transform
     */
    
    printf("\n--- Example 1: Euler to Quaternion Conversion ---\n");
    
    /* Set Euler angles (rotation about X, Y, Z axes) */
    euler[_A_] = 0.0;              /* alpha: rotation about X */
    euler[_B_] = 0.0;              /* beta: rotation about Y */
    euler[_G_] = PI / 4.0;         /* gamma: rotation about Z (45 degrees) */
    
    print_euler(euler, "Input Euler angles");
    
    /* Convert to quaternion using eulerToQuatInv (local->global) */
    SL_quat q1;
    memset(&q1, 0, sizeof(SL_quat));
    eulerToQuatInv(euler, &q1);
    
    print_quat(&q1, "Quaternion (eulerToQuatInv)");
    printf("  Quaternion norm: %.6f (should be 1.0)\n", quat_norm(&q1));
    
    /* Also try eulerToQuat (global->local) */
    SL_quat q2;
    memset(&q2, 0, sizeof(SL_quat));
    eulerToQuat(euler, &q2);
    
    print_quat(&q2, "Quaternion (eulerToQuat)");
    
    /* ----------------------------------------------------------------
     * Example 2: Quaternion to Euler Angles Conversion
     * ----------------------------------------------------------------
     * Convert quaternions back to Euler angles.
     */
    
    printf("\n--- Example 2: Quaternion to Euler Conversion ---\n");
    
    /* Convert quaternion back to Euler angles */
    quatToEulerInv(&q1, euler_back);
    print_euler(euler_back, "Recovered Euler (quatToEulerInv)");
    
    quatToEuler(&q2, euler_back);
    print_euler(euler_back, "Recovered Euler (quatToEuler)");
    
    /* ----------------------------------------------------------------
     * Example 3: Quaternion to Rotation Matrix
     * ----------------------------------------------------------------
     * Convert quaternions to 3x3 rotation matrices.
     */
    
    printf("\n--- Example 3: Quaternion to Rotation Matrix ---\n");
    
    /* Create a quaternion for 90 degree rotation about Z axis */
    euler[_A_] = 0.0;
    euler[_B_] = 0.0;
    euler[_G_] = PI / 2.0;   /* 90 degrees */
    
    SL_quat q_rot90;
    memset(&q_rot90, 0, sizeof(SL_quat));
    eulerToQuatInv(euler, &q_rot90);
    
    print_quat(&q_rot90, "90 deg Z rotation quaternion");
    
    /* Convert to rotation matrix */
    mat_zero(R);
    quatToRotMat(&q_rot90, R);
    print_rotation_matrix(R, "Rotation matrix (global->local)");
    
    mat_zero(R);
    quatToRotMatInv(&q_rot90, R);
    print_rotation_matrix(R, "Rotation matrix (local->global)");
    
    /* ----------------------------------------------------------------
     * Example 4: Quaternion Derivatives
     * ----------------------------------------------------------------
     * Compute quaternion velocity and acceleration from angular velocity.
     */
    
    printf("\n--- Example 4: Quaternion Derivatives ---\n");
    
    SL_quat q_deriv;
    memset(&q_deriv, 0, sizeof(SL_quat));
    
    /* Set a unit quaternion (identity) */
    q_deriv.q[_Q0_] = 1.0;
    q_deriv.q[_Q1_] = 0.0;
    q_deriv.q[_Q2_] = 0.0;
    q_deriv.q[_Q3_] = 0.0;
    
    /* Set angular velocity (rotating about Z axis at 1 rad/s) */
    q_deriv.ad[_A_] = 0.0;
    q_deriv.ad[_B_] = 0.0;
    q_deriv.ad[_G_] = 1.0;   /* 1 rad/s about Z */
    
    /* Set angular acceleration */
    q_deriv.add[_A_] = 0.0;
    q_deriv.add[_B_] = 0.0;
    q_deriv.add[_G_] = 0.1;  /* 0.1 rad/s^2 about Z */
    
    printf("Before quatDerivatives:\n");
    printf("  Angular velocity: [%.4f, %.4f, %.4f] rad/s\n",
           q_deriv.ad[_A_], q_deriv.ad[_B_], q_deriv.ad[_G_]);
    printf("  Quaternion: [w=%.6f, x=%.6f, y=%.6f, z=%.6f]\n",
           q_deriv.q[_Q0_], q_deriv.q[_Q1_], q_deriv.q[_Q2_], q_deriv.q[_Q3_]);
    
    /* Compute quaternion derivatives using SL library function */
    quatDerivatives(&q_deriv);
    
    printf("\nAfter quatDerivatives:\n");
    printf("  Quaternion velocity (qd): [%.6f, %.6f, %.6f, %.6f]\n",
           q_deriv.qd[_Q0_], q_deriv.qd[_Q1_], q_deriv.qd[_Q2_], q_deriv.qd[_Q3_]);
    printf("  Quaternion acceleration (qdd): [%.6f, %.6f, %.6f, %.6f]\n",
           q_deriv.qdd[_Q0_], q_deriv.qdd[_Q1_], q_deriv.qdd[_Q2_], q_deriv.qdd[_Q3_]);
    
    /* ----------------------------------------------------------------
     * Example 5: Angular Velocity from Quaternion
     * ----------------------------------------------------------------
     * Convert quaternion and its derivative back to angular velocity.
     */
    
    printf("\n--- Example 5: Angular Velocity from Quaternion ---\n");
    
    SL_quat q_ang;
    memset(&q_ang, 0, sizeof(SL_quat));
    
    /* Set a unit quaternion and its derivative */
    q_ang.q[_Q0_] = 1.0;
    q_ang.q[_Q1_] = 0.0;
    q_ang.q[_Q2_] = 0.0;
    q_ang.q[_Q3_] = 0.0;
    
    /* Set quaternion velocity (from previous example) */
    q_ang.qd[_Q0_] = 0.0;
    q_ang.qd[_Q1_] = 0.0;
    q_ang.qd[_Q2_] = 0.0;
    q_ang.qd[_Q3_] = 0.5;   /* corresponds to 1 rad/s about Z */
    
    printf("Input quaternion velocity:\n");
    printf("  qd: [%.6f, %.6f, %.6f, %.6f]\n",
           q_ang.qd[_Q0_], q_ang.qd[_Q1_], q_ang.qd[_Q2_], q_ang.qd[_Q3_]);
    
    /* Compute angular velocity using SL library function */
    quatToAngularVelocity(&q_ang);
    
    printf("Recovered angular velocity:\n");
    printf("  ad: [%.6f, %.6f, %.6f] rad/s\n",
           q_ang.ad[_A_], q_ang.ad[_B_], q_ang.ad[_G_]);
    
    /* ----------------------------------------------------------------
     * Example 6: Quaternion Error Computation
     * ----------------------------------------------------------------
     * Compute error between two quaternions (useful for feedback control).
     */
    
    printf("\n--- Example 6: Quaternion Error Computation ---\n");
    
    /* Create two quaternions: current and desired orientation */
    SL_quat q_current, q_desired;
    
    /* Current: identity (no rotation) */
    memset(&q_current, 0, sizeof(SL_quat));
    q_current.q[_Q0_] = 1.0;
    q_current.q[_Q1_] = 0.0;
    q_current.q[_Q2_] = 0.0;
    q_current.q[_Q3_] = 0.0;
    
    /* Desired: 30 degree rotation about Z */
    euler[_A_] = 0.0;
    euler[_B_] = 0.0;
    euler[_G_] = PI / 6.0;   /* 30 degrees */
    
    memset(&q_desired, 0, sizeof(SL_quat));
    eulerToQuatInv(euler, &q_desired);
    
    print_quat(&q_current, "Current orientation");
    print_quat(&q_desired, "Desired orientation");
    
    /* Compute scalar error using SL library function */
    double error = quatError(q_desired.q, q_current.q);
    printf("\nQuaternion error (scalar): %.6f\n", error);
    printf("  (Range: 0 = identical, 1 = max difference)\n");
    
    /* Compute error vector using SL library function (useful for PD control) */
    double error_vec[N_CART+1];
    quatErrorVector(q_desired.q, q_current.q, error_vec);
    printf("\nQuaternion error vector: [%.6f, %.6f, %.6f]\n",
           error_vec[_A_], error_vec[_B_], error_vec[_G_]);
    
    /* ----------------------------------------------------------------
     * Example 7: Rotation Matrix to Euler Angles
     * ----------------------------------------------------------------
     */
    
    printf("\n--- Example 7: Rotation Matrix to Euler Angles ---\n");
    
    /* Create a rotation matrix for 45 deg about Z */
    double angle = PI / 4.0;
    mat_zero(R);
    R[1][1] = cos(angle);  R[1][2] = sin(angle);  R[1][3] = 0.0;
    R[2][1] = -sin(angle); R[2][2] = cos(angle);  R[2][3] = 0.0;
    R[3][1] = 0.0;         R[3][2] = 0.0;         R[3][3] = 1.0;
    
    print_rotation_matrix(R, "Input rotation matrix (45 deg about Z)");
    
    /* Use SL library function to convert back to Euler angles */
    rotMatToEuler(R, euler_back);
    print_euler(euler_back, "Recovered Euler angles");
    
    /* ----------------------------------------------------------------
     * Summary
     * ----------------------------------------------------------------
     */
    
    printf("\n=================================================\n");
    printf("Summary: Quaternion Operations\n");
    printf("=================================================\n");
    printf("\nKey SL library functions demonstrated:\n");
    printf("  - eulerToQuat/eulerToQuatInv: Euler angles to quaternion\n");
    printf("  - quatToEuler/quatToEulerInv: Quaternion to Euler angles\n");
    printf("  - quatToRotMat/quatToRotMatInv: Quaternion to rotation matrix\n");
    printf("  - quatDerivatives: Compute qd and qdd from angular velocity\n");
    printf("  - quatToAngularVelocity: Recover angular velocity from qd\n");
    printf("  - quatError: Scalar error between quaternions\n");
    printf("  - quatErrorVector: Error vector for feedback control\n");
    printf("  - rotMatToEuler: Rotation matrix to Euler angles\n");
    printf("\nConventions:\n");
    printf("  - 'Inv' suffix: local->global transformation\n");
    printf("  - Without 'Inv': global->local transformation\n");
    printf("  - Euler angles: a=X, b=Y, g=Z rotations\n");
    printf("\n");
    
    /* Free allocated matrices and vectors */
    my_free_matrix(R, 1, 3, 1, 3);
    my_free_vector(euler, 1, 3);
    my_free_vector(euler_back, 1, 3);
    
    return 0;
}
