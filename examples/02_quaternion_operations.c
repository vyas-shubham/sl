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

  This is a standalone example that implements the quaternion functions locally.
  In actual SL usage, these would be from SL_common.c.

  ============================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Define PI if not already defined */
#ifndef PI
#define PI 3.14159265358979323846
#endif

/* 
 * SL Index Definitions
 */
#define N_CART 3
#define N_QUAT 4
#define START_INDEX 1

#define _X_ (0+START_INDEX)
#define _Y_ (1+START_INDEX)
#define _Z_ (2+START_INDEX)

#define _A_ (0+START_INDEX)
#define _B_ (1+START_INDEX)
#define _G_ (2+START_INDEX)

#define _Q0_ 1
#define _Q1_ 2
#define _Q2_ 3
#define _Q3_ 4

/**
 * \brief Quaternion orientation structure
 */
typedef struct {
  double   q[N_QUAT+1];    /**< Quaternion [q0,q1,q2,q3] */
  double   qd[N_QUAT+1];   /**< Quaternion velocity */
  double   qdd[N_QUAT+1];  /**< Quaternion acceleration */
  double   ad[N_CART+1];   /**< Angular Velocity [alpha,beta,gamma] */
  double   add[N_CART+1];  /**< Angular Acceleration */
} SL_quat;

/* Helper macro for squaring */
#define sqr(x) ((x)*(x))

/*
 * Quaternion Functions (implementations from SL_common.c)
 */

/**
 * \brief Convert Euler angles to rotation matrix (global->local)
 */
void eulerToRotMat(double *a, double R[4][4]) {
  R[1][1] =  cos(a[2])*cos(a[3]);
  R[1][2] =  cos(a[3])*sin(a[1])*sin(a[2]) + cos(a[1])*sin(a[3]);
  R[1][3] = -(cos(a[1])*cos(a[3])*sin(a[2])) +sin(a[1])*sin(a[3]);
  R[2][1] = -(cos(a[2])*sin(a[3]));
  R[2][2] =  cos(a[1])*cos(a[3]) - sin(a[1])*sin(a[2])*sin(a[3]);
  R[2][3] =  cos(a[3])*sin(a[1]) + cos(a[1])*sin(a[2])*sin(a[3]);
  R[3][1] =  sin(a[2]);
  R[3][2] = -(cos(a[2])*sin(a[1]));
  R[3][3] =  cos(a[1])*cos(a[2]);
}

/**
 * \brief Convert Euler angles to rotation matrix (local->global)
 */
void eulerToRotMatInv(double *a, double R[4][4]) {
  R[1][1] =  cos(a[2])*cos(a[3]);
  R[2][1] =  cos(a[3])*sin(a[1])*sin(a[2]) + cos(a[1])*sin(a[3]);
  R[3][1] = -(cos(a[1])*cos(a[3])*sin(a[2])) +sin(a[1])*sin(a[3]);
  R[1][2] = -(cos(a[2])*sin(a[3]));
  R[2][2] =  cos(a[1])*cos(a[3]) - sin(a[1])*sin(a[2])*sin(a[3]);
  R[3][2] =  cos(a[3])*sin(a[1]) + cos(a[1])*sin(a[2])*sin(a[3]);
  R[1][3] =  sin(a[2]);
  R[2][3] = -(cos(a[2])*sin(a[1]));
  R[3][3] =  cos(a[1])*cos(a[2]);
}

/**
 * \brief Convert rotation matrix to quaternion
 */
void linkQuat(double R[4][4], SL_quat *q) {
  int r;
  double T, q_aux[5];
  double quat_sign;
  double aux;

  T = R[1][1] + R[2][2] + R[3][3];

  if (T > 0.0) {
    T = sqrt(T + 1.0);
    q_aux[1] = 0.5 * T;
    T = 0.5 / T;
    q_aux[2] = (R[3][2] - R[2][3]) * T;
    q_aux[3] = (R[1][3] - R[3][1]) * T;
    q_aux[4] = (R[2][1] - R[1][2]) * T;
  } else {
    int i = 0;
    if (R[2][2] > R[1][1]) i = 1;
    if (R[3][3] > R[i+1][i+1]) i = 2;
    int j = (i + 1) % 3;
    int k = (j + 1) % 3;

    T = sqrt(R[i+1][i+1] - R[j+1][j+1] - R[k+1][k+1] + 1.0);
    q_aux[i+2] = 0.5 * T;
    T = 0.5 / T;
    q_aux[1] = (R[k+1][j+1] - R[j+1][k+1]) * T;
    q_aux[j+2] = (R[j+1][i+1] + R[i+1][j+1]) * T;
    q_aux[k+2] = (R[k+1][i+1] + R[i+1][k+1]) * T;
  }

  /* Check for valid reference quaternion */
  aux = 0.0;
  for (r = 1; r <= N_QUAT; r++)
    aux += sqr(q->q[r]);
  aux = sqrt(aux);

  /* Fix sign of quaternion */
  if (fabs(1.0 - aux) < 0.01) {
    quat_sign = q->q[_Q0_] * q_aux[1] + q->q[_Q1_] * q_aux[2] +
                q->q[_Q2_] * q_aux[3] + q->q[_Q3_] * q_aux[4];
    if (quat_sign < 0.0) {
      q_aux[1] *= -1.0;
      q_aux[2] *= -1.0;
      q_aux[3] *= -1.0;
      q_aux[4] *= -1.0;
    }
  }

  q->q[_Q0_] = q_aux[1];
  q->q[_Q1_] = q_aux[2];
  q->q[_Q2_] = q_aux[3];
  q->q[_Q3_] = q_aux[4];
}

/**
 * \brief Convert Euler angles to quaternion (global->local transform)
 */
void eulerToQuat(double *a, SL_quat *q) {
  double R[4][4];
  memset(R, 0, sizeof(R));
  eulerToRotMatInv(a, R);
  linkQuat(R, q);
}

/**
 * \brief Convert Euler angles to quaternion (local->global transform)
 */
void eulerToQuatInv(double *a, SL_quat *q) {
  double R[4][4];
  memset(R, 0, sizeof(R));
  eulerToRotMat(a, R);
  linkQuat(R, q);
}

/**
 * \brief Convert quaternion to rotation matrix (global->local)
 */
void quatToRotMat(SL_quat *q, double R[4][4]) {
  R[1][1] = -1.0 + 2.0*sqr(q->q[_Q0_]) + 2.0*sqr(q->q[_Q1_]);
  R[2][2] = -1.0 + 2.0*sqr(q->q[_Q0_]) + 2.0*sqr(q->q[_Q2_]);
  R[3][3] = -1.0 + 2.0*sqr(q->q[_Q0_]) + 2.0*sqr(q->q[_Q3_]);
  R[1][2] = 2.0 * (q->q[_Q1_]*q->q[_Q2_] + q->q[_Q0_]*q->q[_Q3_]);
  R[1][3] = 2.0 * (q->q[_Q1_]*q->q[_Q3_] - q->q[_Q0_]*q->q[_Q2_]);
  R[2][1] = 2.0 * (q->q[_Q1_]*q->q[_Q2_] - q->q[_Q0_]*q->q[_Q3_]);
  R[2][3] = 2.0 * (q->q[_Q2_]*q->q[_Q3_] + q->q[_Q0_]*q->q[_Q1_]);
  R[3][1] = 2.0 * (q->q[_Q1_]*q->q[_Q3_] + q->q[_Q0_]*q->q[_Q2_]);
  R[3][2] = 2.0 * (q->q[_Q2_]*q->q[_Q3_] - q->q[_Q0_]*q->q[_Q1_]);
}

/**
 * \brief Convert quaternion to rotation matrix (local->global)
 */
void quatToRotMatInv(SL_quat *q, double R[4][4]) {
  R[1][1] = -1.0 + 2.0*sqr(q->q[_Q0_]) + 2.0*sqr(q->q[_Q1_]);
  R[2][2] = -1.0 + 2.0*sqr(q->q[_Q0_]) + 2.0*sqr(q->q[_Q2_]);
  R[3][3] = -1.0 + 2.0*sqr(q->q[_Q0_]) + 2.0*sqr(q->q[_Q3_]);
  R[2][1] = 2.0 * (q->q[_Q1_]*q->q[_Q2_] + q->q[_Q0_]*q->q[_Q3_]);
  R[3][1] = 2.0 * (q->q[_Q1_]*q->q[_Q3_] - q->q[_Q0_]*q->q[_Q2_]);
  R[1][2] = 2.0 * (q->q[_Q1_]*q->q[_Q2_] - q->q[_Q0_]*q->q[_Q3_]);
  R[3][2] = 2.0 * (q->q[_Q2_]*q->q[_Q3_] + q->q[_Q0_]*q->q[_Q1_]);
  R[1][3] = 2.0 * (q->q[_Q1_]*q->q[_Q3_] + q->q[_Q0_]*q->q[_Q2_]);
  R[2][3] = 2.0 * (q->q[_Q2_]*q->q[_Q3_] - q->q[_Q0_]*q->q[_Q1_]);
}

/**
 * \brief Safe atan2 function
 */
double atan2_safe(double y, double x) {
  if (fabs(x) < 1e-10 && fabs(y) < 1e-10)
    return 0.0;
  return atan2(y, x);
}

/**
 * \brief Convert rotation matrix to Euler angles
 */
void rotMatToEuler(double R[4][4], double *a) {
  a[1] = atan2_safe(R[3][3], -R[3][2]);
  a[2] = atan2_safe(sqrt(sqr(R[3][2]) + sqr(R[3][3])), R[3][1]);
  a[3] = atan2_safe(R[1][1], -R[2][1]);
}

/**
 * \brief Convert rotation matrix to Euler angles (inverse)
 */
void rotMatToEulerInv(double R[4][4], double *a) {
  a[1] = atan2_safe(R[3][3], -R[2][3]);
  a[2] = atan2_safe(sqrt(sqr(R[2][3]) + sqr(R[3][3])), R[1][3]);
  a[3] = atan2_safe(R[1][1], -R[1][2]);
}

/**
 * \brief Convert quaternion to Euler angles
 */
void quatToEuler(SL_quat *q, double *a) {
  double R[4][4];
  memset(R, 0, sizeof(R));
  quatToRotMat(q, R);
  rotMatToEuler(R, a);
}

/**
 * \brief Convert quaternion to Euler angles (inverse)
 */
void quatToEulerInv(SL_quat *q, double *a) {
  double R[4][4];
  memset(R, 0, sizeof(R));
  quatToRotMatInv(q, R);
  rotMatToEulerInv(R, a);
}

/**
 * \brief Compute quaternion derivatives from angular velocity
 */
void quatDerivatives(SL_quat *q) {
  int i, j;
  double Q[5][4];
  double Qd[5][4];

  Q[1][1] = -q->q[_Q1_];
  Q[1][2] = -q->q[_Q2_];
  Q[1][3] = -q->q[_Q3_];
  Q[2][1] =  q->q[_Q0_];
  Q[2][2] =  q->q[_Q3_];
  Q[2][3] = -q->q[_Q2_];
  Q[3][1] = -q->q[_Q3_];
  Q[3][2] =  q->q[_Q0_];
  Q[3][3] =  q->q[_Q1_];
  Q[4][1] =  q->q[_Q2_];
  Q[4][2] = -q->q[_Q1_];
  Q[4][3] =  q->q[_Q0_];

  for (i = 1; i <= N_QUAT; ++i) {
    q->qd[i] = 0.0;
    for (j = 1; j <= N_CART; ++j)
      q->qd[i] += 0.5 * Q[i][j] * q->ad[j];
  }

  Qd[1][1] = -q->qd[_Q1_];
  Qd[1][2] = -q->qd[_Q2_];
  Qd[1][3] = -q->qd[_Q3_];
  Qd[2][1] =  q->qd[_Q0_];
  Qd[2][2] =  q->qd[_Q3_];
  Qd[2][3] = -q->qd[_Q2_];
  Qd[3][1] = -q->qd[_Q3_];
  Qd[3][2] =  q->qd[_Q0_];
  Qd[3][3] =  q->qd[_Q1_];
  Qd[4][1] =  q->qd[_Q2_];
  Qd[4][2] = -q->qd[_Q1_];
  Qd[4][3] =  q->qd[_Q0_];

  for (i = 1; i <= N_QUAT; ++i) {
    q->qdd[i] = 0.0;
    for (j = 1; j <= N_CART; ++j)
      q->qdd[i] += 0.5 * Qd[i][j] * q->ad[j] + 0.5 * Q[i][j] * q->add[j];
  }
}

/**
 * \brief Compute angular velocity from quaternion and its derivative
 */
void quatToAngularVelocity(SL_quat *q) {
  q->ad[_A_] = 2.0 * (-q->q[_Q1_]*q->qd[_Q0_] + q->q[_Q0_]*q->qd[_Q1_]
                      -q->q[_Q3_]*q->qd[_Q2_] + q->q[_Q2_]*q->qd[_Q3_]);
  q->ad[_B_] = 2.0 * (-q->q[_Q2_]*q->qd[_Q0_] + q->q[_Q3_]*q->qd[_Q1_]
                      +q->q[_Q0_]*q->qd[_Q2_] - q->q[_Q1_]*q->qd[_Q3_]);
  q->ad[_G_] = 2.0 * (-q->q[_Q3_]*q->qd[_Q0_] - q->q[_Q2_]*q->qd[_Q1_]
                      +q->q[_Q1_]*q->qd[_Q2_] + q->q[_Q0_]*q->qd[_Q3_]);
}

/**
 * \brief Compute scalar error between two quaternions
 */
double quatError(double *q1, double *q2) {
  double norm = 0;
  double aux;

  aux = q1[_Q0_] * q2[_Q1_] - q2[_Q0_] * q1[_Q1_] + (q1[_Q2_] * q2[_Q3_] - q2[_Q2_] * q1[_Q3_]);
  norm += sqr(aux);

  aux = q1[_Q0_] * q2[_Q2_] - q2[_Q0_] * q1[_Q2_] + (q1[_Q3_] * q2[_Q1_] - q2[_Q3_] * q1[_Q1_]);
  norm += sqr(aux);

  aux = q1[_Q0_] * q2[_Q3_] - q2[_Q0_] * q1[_Q3_] + (q1[_Q1_] * q2[_Q2_] - q2[_Q1_] * q1[_Q2_]);
  norm += sqr(aux);

  return sqrt(norm);
}

/**
 * \brief Compute error vector between quaternions (for feedback control)
 */
void quatErrorVector(double *q1, double *q2, double *ad) {
  ad[_A_] = q1[_Q0_] * q2[_Q1_] - q2[_Q0_] * q1[_Q1_] + (q1[_Q2_] * q2[_Q3_] - q2[_Q2_] * q1[_Q3_]);
  ad[_B_] = q1[_Q0_] * q2[_Q2_] - q2[_Q0_] * q1[_Q2_] + (q1[_Q3_] * q2[_Q1_] - q2[_Q3_] * q1[_Q1_]);
  ad[_G_] = q1[_Q0_] * q2[_Q3_] - q2[_Q0_] * q1[_Q3_] + (q1[_Q1_] * q2[_Q2_] - q2[_Q1_] * q1[_Q2_]);
}

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
 * \brief Print Euler angles
 * 
 * \param[in] angles  Vector of Euler angles [alpha, beta, gamma]
 * \param[in] label   Label for the angles
 */
void print_euler(double *angles, const char *label) {
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
 * \param[in] R       The rotation matrix (1-indexed)
 * \param[in] label   Label for the matrix
 */
void print_rotation_matrix(double R[4][4], const char *label) {
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
    
    /* Allocate rotation matrices (1-indexed, so we need 4x4) */
    double R[4][4];
    double euler[4];  /* 1-indexed Euler angles */
    
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
    
    double euler_back[4];
    
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
    memset(R, 0, sizeof(R));
    quatToRotMat(&q_rot90, R);
    print_rotation_matrix(R, "Rotation matrix (global->local)");
    
    memset(R, 0, sizeof(R));
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
    
    /* Compute quaternion derivatives */
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
    
    /* Compute angular velocity */
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
    
    /* Compute scalar error */
    double error = quatError(q_desired.q, q_current.q);
    printf("\nQuaternion error (scalar): %.6f\n", error);
    printf("  (Range: 0 = identical, 1 = max difference)\n");
    
    /* Compute error vector (useful for PD control) */
    double error_vec[4];
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
    memset(R, 0, sizeof(R));
    R[1][1] = cos(angle);  R[1][2] = sin(angle);  R[1][3] = 0.0;
    R[2][1] = -sin(angle); R[2][2] = cos(angle);  R[2][3] = 0.0;
    R[3][1] = 0.0;         R[3][2] = 0.0;         R[3][3] = 1.0;
    
    print_rotation_matrix(R, "Input rotation matrix (45 deg about Z)");
    
    rotMatToEuler(R, euler_back);
    print_euler(euler_back, "Recovered Euler angles");
    
    /* ----------------------------------------------------------------
     * Summary
     * ----------------------------------------------------------------
     */
    
    printf("\n=================================================\n");
    printf("Summary: Quaternion Operations\n");
    printf("=================================================\n");
    printf("\nKey functions demonstrated:\n");
    printf("  - eulerToQuat/eulerToQuatInv: Euler angles to quaternion\n");
    printf("  - quatToEuler/quatToEulerInv: Quaternion to Euler angles\n");
    printf("  - quatToRotMat/quatToRotMatInv: Quaternion to rotation matrix\n");
    printf("  - quatDerivatives: Compute qd and qdd from angular velocity\n");
    printf("  - quatToAngularVelocity: Recover angular velocity from qd\n");
    printf("  - quatError: Scalar error between quaternions\n");
    printf("  - quatErrorVector: Error vector for feedback control\n");
    printf("\nConventions:\n");
    printf("  - 'Inv' suffix: local->global transformation\n");
    printf("  - Without 'Inv': global->local transformation\n");
    printf("  - Euler angles: Rz * Ry * Rx order (a=X, b=Y, g=Z)\n");
    printf("\n");
    
    return 0;
}
