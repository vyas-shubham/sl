/*!=============================================================================
  ==============================================================================

  \file    01_data_structures.c

  \author  SL Examples
  \date    2024

  ==============================================================================
  \remarks

  Example demonstrating the core data structures used in the SL library.
  
  This example shows how to use:
  - SL_Jstate: Joint space state (position, velocity, acceleration, torque)
  - SL_DJstate: Desired joint state for control
  - SL_Cstate: Cartesian state (position, velocity, acceleration)
  - SL_quat: Quaternion orientation representation
  - SL_link: Link parameters for rigid body dynamics

  This is a standalone example that defines the necessary structures locally.
  In actual SL usage, these would be included from SL.h.

  ============================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* 
 * SL Data Structure Definitions
 * 
 * These structures are defined in SL.h in the actual library.
 * They are replicated here for standalone example compilation.
 */

#define N_CART 3
#define N_QUAT 4
#define START_INDEX 1

/* Cartesian coordinate indices (1-indexed) */
#define _X_ (0+START_INDEX)
#define _Y_ (1+START_INDEX)
#define _Z_ (2+START_INDEX)

/* Euler angle indices */
#define _A_ (0+START_INDEX)
#define _B_ (1+START_INDEX)
#define _G_ (2+START_INDEX)

/* Quaternion indices */
#define _Q0_ 1
#define _Q1_ 2
#define _Q2_ 3
#define _Q3_ 4

/**
 * \brief Joint space state structure
 * 
 * Holds the current state of a robot joint including position,
 * velocity, acceleration, and torque information.
 */
typedef struct {
  double   th;   /**< theta - joint position */
  double   thd;  /**< theta-dot - joint velocity */
  double   thdd; /**< theta-dot-dot - joint acceleration */
  double   ufb;  /**< feedback portion of command */
  double   u;    /**< total torque command */
  double   load; /**< sensed torque */
} SL_Jstate;

/**
 * \brief Desired joint state structure
 * 
 * Holds the desired/target values for joint control.
 */
typedef struct {
  double   th;   /**< desired theta */
  double   thd;  /**< desired theta-dot */
  double   thdd; /**< desired theta-dot-dot */
  double   uff;  /**< feedforward torque command */
  double   uex;  /**< externally imposed torque */
} SL_DJstate;

/**
 * \brief Cartesian state structure
 * 
 * Holds 3D Cartesian position, velocity, and acceleration.
 */
typedef struct {
  double   x[N_CART+1];    /**< Position [x,y,z] */
  double   xd[N_CART+1];   /**< Velocity */
  double   xdd[N_CART+1];  /**< Acceleration */
} SL_Cstate;

/**
 * \brief Quaternion orientation structure
 * 
 * Holds quaternion representation of orientation along with
 * angular velocities and accelerations.
 */
typedef struct {
  double   q[N_QUAT+1];    /**< Position [q0,q1,q2,q3] */
  double   qd[N_QUAT+1];   /**< Velocity */
  double   qdd[N_QUAT+1];  /**< Acceleration */
  double   ad[N_CART+1];   /**< Angular Velocity [alpha,beta,gamma] */
  double   add[N_CART+1];  /**< Angular Acceleration */
} SL_quat;

/**
 * \brief Link parameters structure
 * 
 * Contains rigid body dynamics parameters for a robot link.
 */
typedef struct {
  double   m;                              /**< Mass */
  double   mcm[N_CART+1];                  /**< Center of mass times mass */
  double   inertia[N_CART+1][N_CART+1];    /**< Moment of inertia */
  double   vis;                            /**< viscous friction term */
  double   coul;                           /**< coulomb friction */
  double   stiff;                          /**< spring stiffness */
  double   cons;                           /**< constant term */
} SL_link;

/**
 * \brief Print joint state information
 * 
 * Demonstrates accessing SL_Jstate structure members
 * 
 * \param[in] state    The joint state to print
 * \param[in] name     Name of the joint for display
 */
void print_joint_state(SL_Jstate *state, const char *name) {
    printf("\nJoint State for %s:\n", name);
    printf("  Position (th):     %8.4f rad\n", state->th);
    printf("  Velocity (thd):    %8.4f rad/s\n", state->thd);
    printf("  Acceleration (thdd): %8.4f rad/s^2\n", state->thdd);
    printf("  Feedback torque (ufb): %8.4f Nm\n", state->ufb);
    printf("  Command torque (u):    %8.4f Nm\n", state->u);
    printf("  Sensed load (load):    %8.4f Nm\n", state->load);
}

/**
 * \brief Print desired joint state information
 * 
 * Demonstrates accessing SL_DJstate structure members
 * 
 * \param[in] state    The desired joint state to print
 * \param[in] name     Name of the joint for display
 */
void print_desired_joint_state(SL_DJstate *state, const char *name) {
    printf("\nDesired Joint State for %s:\n", name);
    printf("  Desired position (th):     %8.4f rad\n", state->th);
    printf("  Desired velocity (thd):    %8.4f rad/s\n", state->thd);
    printf("  Desired acceleration (thdd): %8.4f rad/s^2\n", state->thdd);
    printf("  Feedforward torque (uff):    %8.4f Nm\n", state->uff);
    printf("  External torque (uex):       %8.4f Nm\n", state->uex);
}

/**
 * \brief Print Cartesian state information
 * 
 * Demonstrates accessing SL_Cstate structure members
 * 
 * \param[in] state    The Cartesian state to print
 * \param[in] name     Name of the point for display
 */
void print_cartesian_state(SL_Cstate *state, const char *name) {
    printf("\nCartesian State for %s:\n", name);
    printf("  Position:     [%8.4f, %8.4f, %8.4f] m\n", 
           state->x[_X_], state->x[_Y_], state->x[_Z_]);
    printf("  Velocity:     [%8.4f, %8.4f, %8.4f] m/s\n", 
           state->xd[_X_], state->xd[_Y_], state->xd[_Z_]);
    printf("  Acceleration: [%8.4f, %8.4f, %8.4f] m/s^2\n", 
           state->xdd[_X_], state->xdd[_Y_], state->xdd[_Z_]);
}

/**
 * \brief Print quaternion orientation information
 * 
 * Demonstrates accessing SL_quat structure members
 * 
 * \param[in] quat     The quaternion to print
 * \param[in] name     Name of the orientation for display
 */
void print_quaternion(SL_quat *quat, const char *name) {
    printf("\nQuaternion Orientation for %s:\n", name);
    printf("  q = [%8.4f, %8.4f, %8.4f, %8.4f] (w, x, y, z)\n",
           quat->q[_Q0_], quat->q[_Q1_], quat->q[_Q2_], quat->q[_Q3_]);
    printf("  Angular velocity:     [%8.4f, %8.4f, %8.4f] rad/s\n",
           quat->ad[_A_], quat->ad[_B_], quat->ad[_G_]);
    printf("  Angular acceleration: [%8.4f, %8.4f, %8.4f] rad/s^2\n",
           quat->add[_A_], quat->add[_B_], quat->add[_G_]);
}

/**
 * \brief Print link parameters
 * 
 * Demonstrates accessing SL_link structure members
 * 
 * \param[in] link     The link parameters to print
 * \param[in] name     Name of the link for display
 */
void print_link_parameters(SL_link *link, const char *name) {
    printf("\nLink Parameters for %s:\n", name);
    printf("  Mass:    %8.4f kg\n", link->m);
    printf("  Center of mass (mcm): [%8.4f, %8.4f, %8.4f] kg*m\n",
           link->mcm[_X_], link->mcm[_Y_], link->mcm[_Z_]);
    printf("  Inertia tensor (diagonal): [%8.4f, %8.4f, %8.4f] kg*m^2\n",
           link->inertia[_X_][_X_], link->inertia[_Y_][_Y_], link->inertia[_Z_][_Z_]);
    printf("  Viscous friction: %8.4f Nm*s/rad\n", link->vis);
    printf("  Coulomb friction: %8.4f Nm\n", link->coul);
}

/**
 * \brief Main function demonstrating SL data structures
 * 
 * This example shows how to:
 * 1. Initialize and populate joint states
 * 2. Work with Cartesian states
 * 3. Use quaternions for orientation
 * 4. Set up link parameters for dynamics
 */
int main(int argc, char **argv) {
    
    printf("=================================================\n");
    printf("SL Library Example: Core Data Structures\n");
    printf("=================================================\n");
    
    /* ----------------------------------------------------------------
     * Example 1: Joint State (SL_Jstate)
     * ----------------------------------------------------------------
     * SL_Jstate holds the current state of a joint including
     * position, velocity, acceleration, and torque information.
     */
    
    printf("\n--- Example 1: Joint State (SL_Jstate) ---\n");
    
    SL_Jstate joint;
    memset(&joint, 0, sizeof(SL_Jstate));
    
    /* Set example values for a robot joint */
    joint.th   = 0.785;    /* 45 degrees in radians */
    joint.thd  = 0.1;      /* slow motion */
    joint.thdd = 0.0;      /* no acceleration */
    joint.ufb  = 5.0;      /* feedback torque */
    joint.u    = 5.5;      /* total command torque */
    joint.load = 4.8;      /* sensed load */
    
    print_joint_state(&joint, "Elbow");
    
    /* ----------------------------------------------------------------
     * Example 2: Desired Joint State (SL_DJstate)
     * ----------------------------------------------------------------
     * SL_DJstate holds the desired/target values for control.
     */
    
    printf("\n--- Example 2: Desired Joint State (SL_DJstate) ---\n");
    
    SL_DJstate desired;
    memset(&desired, 0, sizeof(SL_DJstate));
    
    /* Set target state for the joint */
    desired.th   = 1.57;   /* 90 degrees target */
    desired.thd  = 0.0;    /* zero velocity at target */
    desired.thdd = 0.0;    /* zero acceleration at target */
    desired.uff  = 2.0;    /* feedforward torque for gravity compensation */
    desired.uex  = 0.0;    /* no external torque */
    
    print_desired_joint_state(&desired, "Elbow");
    
    /* ----------------------------------------------------------------
     * Example 3: Cartesian State (SL_Cstate)
     * ----------------------------------------------------------------
     * SL_Cstate holds 3D Cartesian position, velocity, and acceleration.
     */
    
    printf("\n--- Example 3: Cartesian State (SL_Cstate) ---\n");
    
    SL_Cstate cart;
    memset(&cart, 0, sizeof(SL_Cstate));
    
    /* Set end-effector position */
    cart.x[_X_] = 0.5;     /* 0.5 meters in X */
    cart.x[_Y_] = 0.0;     /* centered in Y */
    cart.x[_Z_] = 0.3;     /* 0.3 meters in Z */
    
    /* Set end-effector velocity */
    cart.xd[_X_] = 0.1;    /* moving in X direction */
    cart.xd[_Y_] = 0.0;
    cart.xd[_Z_] = 0.0;
    
    /* Set end-effector acceleration */
    cart.xdd[_X_] = 0.0;
    cart.xdd[_Y_] = 0.0;
    cart.xdd[_Z_] = 0.0;
    
    print_cartesian_state(&cart, "End Effector");
    
    /* ----------------------------------------------------------------
     * Example 4: Quaternion Orientation (SL_quat)
     * ----------------------------------------------------------------
     * SL_quat represents orientation using quaternions along with
     * angular velocities and accelerations.
     */
    
    printf("\n--- Example 4: Quaternion Orientation (SL_quat) ---\n");
    
    SL_quat orient;
    memset(&orient, 0, sizeof(SL_quat));
    
    /* Set identity quaternion (no rotation) */
    orient.q[_Q0_] = 1.0;  /* w component */
    orient.q[_Q1_] = 0.0;  /* x component */
    orient.q[_Q2_] = 0.0;  /* y component */
    orient.q[_Q3_] = 0.0;  /* z component */
    
    /* Set angular velocity (rotating around Z axis) */
    orient.ad[_A_] = 0.0;  /* alpha (rotation about X) */
    orient.ad[_B_] = 0.0;  /* beta (rotation about Y) */
    orient.ad[_G_] = 0.5;  /* gamma (rotation about Z) */
    
    /* Set angular acceleration */
    orient.add[_A_] = 0.0;
    orient.add[_B_] = 0.0;
    orient.add[_G_] = 0.0;
    
    print_quaternion(&orient, "Tool Frame");
    
    /* ----------------------------------------------------------------
     * Example 5: Link Parameters (SL_link)
     * ----------------------------------------------------------------
     * SL_link contains rigid body dynamics parameters for each link.
     */
    
    printf("\n--- Example 5: Link Parameters (SL_link) ---\n");
    
    SL_link link;
    memset(&link, 0, sizeof(SL_link));
    
    /* Set link mass properties */
    link.m = 2.5;                    /* 2.5 kg */
    link.mcm[_X_] = 0.125;           /* mass * center of mass X */
    link.mcm[_Y_] = 0.0;
    link.mcm[_Z_] = 0.0;
    
    /* Set inertia tensor (diagonal approximation) */
    link.inertia[_X_][_X_] = 0.01;   /* Ixx */
    link.inertia[_Y_][_Y_] = 0.08;   /* Iyy */
    link.inertia[_Z_][_Z_] = 0.08;   /* Izz */
    
    /* Set friction parameters */
    link.vis  = 0.1;                 /* viscous friction coefficient */
    link.coul = 0.05;                /* Coulomb friction */
    
    print_link_parameters(&link, "Upper Arm");
    
    /* ----------------------------------------------------------------
     * Summary
     * ----------------------------------------------------------------
     */
    
    printf("\n=================================================\n");
    printf("Summary: SL Core Data Structures\n");
    printf("=================================================\n");
    printf("\nKey data structures demonstrated:\n");
    printf("  - SL_Jstate:  Joint state (current)\n");
    printf("  - SL_DJstate: Desired joint state (target)\n");
    printf("  - SL_Cstate:  Cartesian state\n");
    printf("  - SL_quat:    Quaternion orientation\n");
    printf("  - SL_link:    Link dynamics parameters\n");
    printf("\nIndex conventions:\n");
    printf("  - _X_, _Y_, _Z_ for Cartesian coordinates (1, 2, 3)\n");
    printf("  - _Q0_, _Q1_, _Q2_, _Q3_ for quaternion (1, 2, 3, 4)\n");
    printf("  - _A_, _B_, _G_ for Euler angles (1, 2, 3)\n");
    printf("\n");
    
    return 0;
}
