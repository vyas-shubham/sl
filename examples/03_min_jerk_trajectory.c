/*!=============================================================================
  ==============================================================================

  \file    03_min_jerk_trajectory.c

  \author  SL Examples
  \date    2024

  ==============================================================================
  \remarks

  Example demonstrating minimum jerk trajectory generation as used in SL.
  
  The minimum jerk trajectory is a smooth trajectory that minimizes the
  integral of squared jerk (third derivative of position). It is commonly
  used in robotics for point-to-point movements as it produces smooth,
  human-like motion.

  The trajectory is computed as a 5th order polynomial:
    x(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5

  This example shows how to:
  - Generate minimum jerk trajectories for joint movements
  - Compute position, velocity, and acceleration profiles
  - Handle multiple degrees of freedom
  - Use SL data structures (SL_DJstate) for trajectory representation

  Compile with:
    gcc -I$LAB_ROOT/include -I../include -L$LAB_LIBDIR -o 03_min_jerk_trajectory \
        03_min_jerk_trajectory.c -lSLcommon -lSLtask -lutility -lm

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// SL specific headers
#include "SL.h"
#include "utility.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif

/* Simulated number of DOFs for this example */
#define EXAMPLE_N_DOFS 3

/**
 * \brief Compute min-jerk trajectory coefficients
 * 
 * For a minimum jerk trajectory from (x0, v0, a0) to (xf, vf, af) over time T,
 * the position is given by:
 *   x(t) = x0 + v0*t + 0.5*a0*t^2 + c3*t^3 + c4*t^4 + c5*t^5
 * 
 * where c3, c4, c5 are computed to satisfy boundary conditions.
 * 
 * \param[in] x0   Initial position
 * \param[in] v0   Initial velocity
 * \param[in] a0   Initial acceleration
 * \param[in] xf   Final position
 * \param[in] vf   Final velocity (typically 0)
 * \param[in] af   Final acceleration (typically 0)
 * \param[in] T    Total trajectory time
 * \param[out] c   Coefficient array [c0, c1, c2, c3, c4, c5]
 */
void compute_min_jerk_coefficients(double x0, double v0, double a0,
                                   double xf, double vf, double af,
                                   double T, double *c) {
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    
    /* c0, c1, c2 are determined by initial conditions */
    c[0] = x0;
    c[1] = v0;
    c[2] = a0 / 2.0;
    
    /* c3, c4, c5 satisfy final boundary conditions */
    double d = xf - x0 - v0 * T - (a0 / 2.0) * T2;
    double dv = vf - v0 - a0 * T;
    double da = af - a0;
    
    /* Solve the linear system for c3, c4, c5 */
    c[3] = (10.0 * d / T3) - (4.0 * dv / T2) + (0.5 * da / T);
    c[4] = (-15.0 * d / T4) + (7.0 * dv / T3) - (1.0 * da / T2);
    c[5] = (6.0 * d / T5) - (3.0 * dv / T4) + (0.5 * da / T3);
}

/**
 * \brief Evaluate min-jerk trajectory at time t
 * 
 * \param[in] c    Coefficient array [c0, c1, c2, c3, c4, c5]
 * \param[in] t    Current time (0 <= t <= T)
 * \param[out] x   Position at time t
 * \param[out] xd  Velocity at time t
 * \param[out] xdd Acceleration at time t
 */
void eval_min_jerk(double *c, double t, double *x, double *xd, double *xdd) {
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    
    *x = c[0] + c[1] * t + c[2] * t2 + c[3] * t3 + c[4] * t4 + c[5] * t5;
    *xd = c[1] + 2.0 * c[2] * t + 3.0 * c[3] * t2 + 4.0 * c[4] * t3 + 5.0 * c[5] * t4;
    *xdd = 2.0 * c[2] + 6.0 * c[3] * t + 12.0 * c[4] * t2 + 20.0 * c[5] * t3;
}

/**
 * \brief Compute minimum jerk trajectory step (SL-style)
 * 
 * This function implements the same algorithm used in SL's goto_task.
 * Given the current state and goal, it computes the next step using
 * minimum jerk trajectory.
 * 
 * \param[in,out] state   Current desired state (position, velocity, acceleration)
 * \param[in]     goal    Goal state
 * \param[in]     tau     Time remaining to reach goal
 * \param[in]     dt      Time step (1/servo_rate)
 * \return                TRUE if successful, FALSE if tau < dt
 */
int calculate_min_jerk_next_step(SL_DJstate *state, SL_DJstate *goal,
                                  double tau, double dt) {
    double t1, t2, t3, t4, t5;
    double tau1, tau2, tau3, tau4, tau5;
    
    if (dt > tau || dt <= 0) {
        return FALSE;
    }
    
    t1 = dt;
    t2 = t1 * dt;
    t3 = t2 * dt;
    t4 = t3 * dt;
    t5 = t4 * dt;
    
    tau1 = tau;
    tau2 = tau1 * tau;
    tau3 = tau2 * tau;
    tau4 = tau3 * tau;
    tau5 = tau4 * tau;
    
    /* Calculate the polynomial coefficients */
    double dist = goal->th - state->th;
    double a1t2 = goal->thdd;
    double a0t2 = state->thdd;
    double v1t1 = goal->thd;
    double v0t1 = state->thd;
    
    double c1 = 6.0 * dist / tau5 + (a1t2 - a0t2) / (2.0 * tau3) - 
                3.0 * (v0t1 + v1t1) / tau4;
    double c2 = -15.0 * dist / tau4 + (3.0 * a0t2 - 2.0 * a1t2) / (2.0 * tau2) +
                (8.0 * v0t1 + 7.0 * v1t1) / tau3;
    double c3 = 10.0 * dist / tau3 + (a1t2 - 3.0 * a0t2) / (2.0 * tau) -
                (6.0 * v0t1 + 4.0 * v1t1) / tau2;
    double c4 = state->thdd / 2.0;
    double c5 = state->thd;
    double c6 = state->th;
    
    state->th = c1 * t5 + c2 * t4 + c3 * t3 + c4 * t2 + c5 * t1 + c6;
    state->thd = 5.0 * c1 * t4 + 4.0 * c2 * t3 + 3.0 * c3 * t2 + 2.0 * c4 * t1 + c5;
    state->thdd = 20.0 * c1 * t3 + 12.0 * c2 * t2 + 6.0 * c3 * t1 + 2.0 * c4;
    
    return TRUE;
}

/**
 * \brief Main function demonstrating minimum jerk trajectories
 */
int main(int argc, char **argv) {
    
    printf("=================================================\n");
    printf("SL Library Example: Minimum Jerk Trajectory\n");
    printf("=================================================\n");
    
    /* ----------------------------------------------------------------
     * Example 1: Basic Min-Jerk Trajectory
     * ----------------------------------------------------------------
     * Generate a minimum jerk trajectory from position 0 to 1 over 2 seconds.
     */
    
    printf("\n--- Example 1: Basic Min-Jerk Trajectory ---\n");
    
    double x0 = 0.0;      /* Initial position */
    double v0 = 0.0;      /* Initial velocity */
    double a0 = 0.0;      /* Initial acceleration */
    double xf = 1.0;      /* Final position */
    double vf = 0.0;      /* Final velocity */
    double af = 0.0;      /* Final acceleration */
    double T = 2.0;       /* Total time */
    
    double c[6];
    compute_min_jerk_coefficients(x0, v0, a0, xf, vf, af, T, c);
    
    printf("\nTrajectory parameters:\n");
    printf("  Initial: x=%.2f, v=%.2f, a=%.2f\n", x0, v0, a0);
    printf("  Final:   x=%.2f, v=%.2f, a=%.2f\n", xf, vf, af);
    printf("  Duration: %.2f seconds\n", T);
    
    printf("\nPolynomial coefficients:\n");
    printf("  c0=%.6f, c1=%.6f, c2=%.6f\n", c[0], c[1], c[2]);
    printf("  c3=%.6f, c4=%.6f, c5=%.6f\n", c[3], c[4], c[5]);
    
    printf("\nTrajectory profile (every 0.2s):\n");
    printf("  Time      Position   Velocity   Acceleration\n");
    printf("  ----------------------------------------\n");
    
    double t;
    for (t = 0.0; t <= T + 0.001; t += 0.2) {
        double x, xd, xdd;
        eval_min_jerk(c, t, &x, &xd, &xdd);
        printf("  %5.2f s   %8.5f   %8.5f   %8.5f\n", t, x, xd, xdd);
    }
    
    /* ----------------------------------------------------------------
     * Example 2: Joint Trajectory using SL Data Structures
     * ----------------------------------------------------------------
     * Use the iterative approach similar to SL's goto_task with SL_DJstate.
     */
    
    printf("\n--- Example 2: Iterative Min-Jerk with SL_DJstate ---\n");
    
    SL_DJstate state;
    SL_DJstate goal;
    
    /* Initialize current state */
    memset(&state, 0, sizeof(SL_DJstate));
    state.th   = 0.0;
    state.thd  = 0.0;
    state.thdd = 0.0;
    
    /* Initialize goal state (90 degrees) */
    memset(&goal, 0, sizeof(SL_DJstate));
    goal.th   = PI / 2;
    goal.thd  = 0.0;
    goal.thdd = 0.0;
    
    double total_time = 1.0;  /* 1 second movement */
    double servo_rate = 100;  /* 100 Hz servo rate */
    double time_step = 1.0 / servo_rate;
    
    printf("\nJoint trajectory: 0 -> 90 degrees in 1 second\n");
    printf("  Servo rate: %.0f Hz\n", servo_rate);
    
    printf("\nTrajectory profile (every 0.1s):\n");
    printf("  Time      Position(deg)   Velocity(deg/s)\n");
    printf("  ------------------------------------------\n");
    
    double time_to_go = total_time;
    int step = 0;
    int print_interval = 10;  /* Print every 10 steps (0.1s) */
    
    while (time_to_go > time_step) {
        if (step % print_interval == 0) {
            double elapsed = total_time - time_to_go;
            printf("  %5.2f s   %10.4f      %10.4f\n", 
                   elapsed, state.th * 180.0 / PI, state.thd * 180.0 / PI);
        }
        
        calculate_min_jerk_next_step(&state, &goal, time_to_go, time_step);
        time_to_go -= time_step;
        step++;
    }
    
    /* Print final state */
    printf("  %5.2f s   %10.4f      %10.4f\n", 
           total_time, state.th * 180.0 / PI, state.thd * 180.0 / PI);
    
    printf("\nFinal state:\n");
    printf("  Position: %.4f rad (%.2f deg)\n", state.th, state.th * 180.0 / PI);
    printf("  Velocity: %.4f rad/s (%.2f deg/s)\n", state.thd, state.thd * 180.0 / PI);
    printf("  Acceleration: %.4f rad/s^2\n", state.thdd);
    
    /* ----------------------------------------------------------------
     * Example 3: Multi-DOF Trajectory with SL_DJstate Array
     * ----------------------------------------------------------------
     * Generate coordinated trajectories for multiple joints using
     * an array of SL_DJstate structures (similar to joint_des_state in SL).
     */
    
    printf("\n--- Example 3: Multi-DOF Coordinated Movement ---\n");
    
    /* Allocate arrays similar to how SL does it (1-indexed) */
    SL_DJstate *joint_des_state;
    SL_DJstate *joint_goal_state;
    
    joint_des_state = (SL_DJstate *)calloc(EXAMPLE_N_DOFS + 1, sizeof(SL_DJstate));
    if (joint_des_state == NULL) {
        printf("Error: Memory allocation failed for joint_des_state\n");
        return 1;
    }
    
    joint_goal_state = (SL_DJstate *)calloc(EXAMPLE_N_DOFS + 1, sizeof(SL_DJstate));
    if (joint_goal_state == NULL) {
        printf("Error: Memory allocation failed for joint_goal_state\n");
        free(joint_des_state);
        return 1;
    }
    
    /* Initialize states (note: 1-indexed like SL) */
    double targets[] = {0, PI / 4, PI / 3, PI / 6};  /* 45, 60, 30 degrees */
    char *joint_names[] = {"", "Shoulder", "Elbow", "Wrist"};  /* 1-indexed */
    
    int i;
    for (i = 1; i <= EXAMPLE_N_DOFS; i++) {
        joint_des_state[i].th = 0.0;
        joint_des_state[i].thd = 0.0;
        joint_des_state[i].thdd = 0.0;
        joint_des_state[i].uff = 0.0;
        
        joint_goal_state[i].th = targets[i];
        joint_goal_state[i].thd = 0.0;
        joint_goal_state[i].thdd = 0.0;
        joint_goal_state[i].uff = 0.0;
    }
    
    total_time = 1.5;  /* 1.5 seconds for all joints */
    time_step = 0.01;  /* 100 Hz */
    
    printf("\nCoordinated 3-DOF movement:\n");
    printf("  Shoulder: 0 -> 45 degrees\n");
    printf("  Elbow:    0 -> 60 degrees\n");
    printf("  Wrist:    0 -> 30 degrees\n");
    printf("  Duration: 1.5 seconds\n");
    
    /* Run the trajectory for all joints */
    time_to_go = total_time;
    while (time_to_go > time_step) {
        for (i = 1; i <= EXAMPLE_N_DOFS; i++) {
            calculate_min_jerk_next_step(&joint_des_state[i], &joint_goal_state[i],
                                         time_to_go, time_step);
        }
        time_to_go -= time_step;
    }
    
    printf("\nFinal state comparison:\n");
    printf("  Joint      Target(deg)  Achieved(deg)  Error(deg)\n");
    printf("  ------------------------------------------------\n");
    
    for (i = 1; i <= EXAMPLE_N_DOFS; i++) {
        double target_deg = joint_goal_state[i].th * 180.0 / PI;
        double achieved_deg = joint_des_state[i].th * 180.0 / PI;
        double error_deg = target_deg - achieved_deg;
        printf("  %-10s %10.4f   %10.4f     %10.6f\n",
               joint_names[i], target_deg, achieved_deg, error_deg);
    }
    
    /* ----------------------------------------------------------------
     * Example 4: Trajectory with Non-zero Initial Velocity
     * ----------------------------------------------------------------
     * Demonstrate blending from a moving state.
     */
    
    printf("\n--- Example 4: Blending with Non-zero Initial Velocity ---\n");
    
    state.th = 0.5;          /* Starting at 0.5 rad */
    state.thd = 0.2;         /* Already moving at 0.2 rad/s */
    state.thdd = 0.0;
    goal.th = 1.5;           /* Target position */
    goal.thd = 0.0;
    goal.thdd = 0.0;
    total_time = 1.0;
    
    printf("\nBlending trajectory:\n");
    printf("  Initial position: 0.5 rad\n");
    printf("  Initial velocity: 0.2 rad/s\n");
    printf("  Target position: 1.5 rad\n");
    printf("  Duration: 1.0 second\n");
    
    /* Compute coefficients for analysis */
    compute_min_jerk_coefficients(state.th, state.thd, state.thdd, 
                                  goal.th, goal.thd, goal.thdd, 
                                  total_time, c);
    
    printf("\nTrajectory profile:\n");
    printf("  Time      Position   Velocity   Acceleration\n");
    printf("  ----------------------------------------\n");
    
    for (t = 0.0; t <= total_time + 0.001; t += 0.1) {
        double x, xd, xdd;
        eval_min_jerk(c, t, &x, &xd, &xdd);
        printf("  %5.2f s   %8.5f   %8.5f   %8.5f\n", t, x, xd, xdd);
    }
    
    /* ----------------------------------------------------------------
     * Summary
     * ----------------------------------------------------------------
     */
    
    printf("\n=================================================\n");
    printf("Summary: Minimum Jerk Trajectory Generation\n");
    printf("=================================================\n");
    printf("\nKey concepts:\n");
    printf("  - Min-jerk minimizes integral of squared jerk\n");
    printf("  - Results in smooth, bell-shaped velocity profile\n");
    printf("  - Commonly used for point-to-point robot movements\n");
    printf("\nTrajectory equation:\n");
    printf("  x(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5\n");
    printf("\nBoundary conditions:\n");
    printf("  - Matches position, velocity, acceleration at start/end\n");
    printf("  - Typically: zero velocity and acceleration at endpoints\n");
    printf("\nSL implementation:\n");
    printf("  - Used in goto_task for smooth joint movements\n");
    printf("  - Computed iteratively at each servo cycle\n");
    printf("  - Uses SL_DJstate for desired state representation\n");
    printf("\n");
    
    /* Free allocated memory */
    free(joint_des_state);
    free(joint_goal_state);
    
    return 0;
}
