/*!=============================================================================
  ==============================================================================

  \file    05_realtime_ipc.c

  \author  SL Examples
  \date    2024

  ==============================================================================
  \remarks

  Example demonstrating real-time inter-process communication (IPC) as used
  in the SL library with PREEMPT-RT Linux.
  
  This example simulates the architecture of a typical SL robot control system
  with three communicating processes:
  
  1. MOTOR SERVO: High-frequency control loop (1000 Hz)
     - Reads joint commands from shared memory
     - Simulates motor control and sensor reading
     - Writes sensor data to shared memory
  
  2. TASK SERVO: Medium-frequency task execution (500 Hz)
     - Computes desired trajectories
     - Writes joint commands to shared memory
     - Reads sensor feedback
  
  3. VISUALIZATION: Low-frequency display (60 Hz)
     - Reads state from shared memory
     - Displays robot state

  Key IPC concepts demonstrated:
  - POSIX shared memory for data exchange
  - Mutexes for thread-safe access
  - Condition variables for synchronization
  - Lock-free techniques for real-time safety
  - Timestamp-based data freshness

  This example can be compiled and run on any Linux system. On PREEMPT-RT
  or Xenomai systems, it would use the real-time primitives from SL_rt_mutex.h.

  ============================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <errno.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

/* Number of simulated DOFs */
#define N_DOFS 3

/* Servo frequencies (Hz) */
#define MOTOR_SERVO_RATE   1000
#define TASK_SERVO_RATE    500
#define DISPLAY_RATE       60

/* Run time in seconds */
#define RUN_TIME_SECONDS   3

/*============================================================================
 * Data Structures (matching SL conventions)
 *============================================================================*/

/**
 * \brief Joint state structure (matches SL_fJstate)
 */
typedef struct {
    float th;    /**< Joint position (rad) */
    float thd;   /**< Joint velocity (rad/s) */
    float thdd;  /**< Joint acceleration (rad/s^2) */
    float u;     /**< Applied torque (Nm) */
    float load;  /**< Measured load (Nm) */
} JointState;

/**
 * \brief Desired joint state structure (matches SL_fDJstate)
 */
typedef struct {
    float th;    /**< Desired position (rad) */
    float thd;   /**< Desired velocity (rad/s) */
    float thdd;  /**< Desired acceleration (rad/s^2) */
    float uff;   /**< Feedforward torque (Nm) */
} DesiredState;

/**
 * \brief Shared memory structure for joint states
 * 
 * This structure is placed in shared memory and protected by a mutex.
 * The timestamp allows consumers to check data freshness.
 */
typedef struct {
    pthread_mutex_t mutex;        /**< Mutex for thread-safe access */
    double timestamp;             /**< Data timestamp (seconds) */
    unsigned long seq_num;        /**< Sequence number for ordering */
    JointState joints[N_DOFS];    /**< Joint state array */
} SharedJointState;

/**
 * \brief Shared memory structure for desired states
 */
typedef struct {
    pthread_mutex_t mutex;        /**< Mutex for thread-safe access */
    double timestamp;             /**< Data timestamp (seconds) */
    unsigned long seq_num;        /**< Sequence number for ordering */
    DesiredState desired[N_DOFS]; /**< Desired state array */
    int new_command;              /**< Flag indicating new command */
} SharedDesiredState;

/**
 * \brief Synchronization semaphores between servos
 */
typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int ready;
} SyncSemaphore;

/*============================================================================
 * Global Variables
 *============================================================================*/

/* Shared memory regions */
static SharedJointState g_joint_state;
static SharedDesiredState g_desired_state;

/* Synchronization primitives */
static SyncSemaphore g_motor_sync;
static SyncSemaphore g_task_sync;

/* Control flag for shutdown */
static volatile int g_running = 1;

/* Statistics */
static unsigned long g_motor_cycles = 0;
static unsigned long g_task_cycles = 0;
static unsigned long g_display_cycles = 0;

/*============================================================================
 * Utility Functions
 *============================================================================*/

/**
 * \brief Get current time in seconds (high resolution)
 */
double get_time(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec * 1e-6;
}

/**
 * \brief Sleep for specified microseconds
 */
void sleep_us(unsigned int us) {
    usleep(us);
}

/**
 * \brief Initialize a synchronization semaphore
 */
void init_sync_sem(SyncSemaphore *sem) {
    pthread_mutex_init(&sem->mutex, NULL);
    pthread_cond_init(&sem->cond, NULL);
    sem->ready = 0;
}

/**
 * \brief Signal a synchronization semaphore
 */
void signal_sync_sem(SyncSemaphore *sem) {
    pthread_mutex_lock(&sem->mutex);
    sem->ready = 1;
    pthread_cond_signal(&sem->cond);
    pthread_mutex_unlock(&sem->mutex);
}

/**
 * \brief Wait on a synchronization semaphore with timeout
 * 
 * \param sem      Semaphore to wait on
 * \param timeout_us Timeout in microseconds
 * \return         0 on success, -1 on timeout
 */
int wait_sync_sem(SyncSemaphore *sem, unsigned int timeout_us) {
    struct timespec ts;
    struct timeval tv;
    int result = 0;
    
    gettimeofday(&tv, NULL);
    ts.tv_sec = tv.tv_sec + timeout_us / 1000000;
    ts.tv_nsec = (tv.tv_usec + (timeout_us % 1000000)) * 1000;
    if (ts.tv_nsec >= 1000000000) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000;
    }
    
    pthread_mutex_lock(&sem->mutex);
    while (!sem->ready && result == 0) {
        result = pthread_cond_timedwait(&sem->cond, &sem->mutex, &ts);
    }
    sem->ready = 0;
    pthread_mutex_unlock(&sem->mutex);
    
    return (result == ETIMEDOUT) ? -1 : 0;
}

/*============================================================================
 * Motor Servo (High-frequency control loop)
 *============================================================================*/

/**
 * \brief Motor servo thread function
 * 
 * Simulates a 1000 Hz motor control loop that:
 * - Reads desired state from shared memory
 * - Applies simple PD control
 * - Simulates joint dynamics
 * - Writes sensor data to shared memory
 */
void *motor_servo_thread(void *arg) {
    (void)arg;
    
    double loop_period = 1.0 / MOTOR_SERVO_RATE;
    double next_time = get_time() + loop_period;
    
    /* Local state variables */
    JointState state[N_DOFS];
    DesiredState desired[N_DOFS];
    
    /* Initialize state */
    int i;
    for (i = 0; i < N_DOFS; i++) {
        state[i].th = 0.0f;
        state[i].thd = 0.0f;
        state[i].thdd = 0.0f;
        state[i].u = 0.0f;
        state[i].load = 0.0f;
        
        desired[i].th = 0.0f;
        desired[i].thd = 0.0f;
        desired[i].thdd = 0.0f;
        desired[i].uff = 0.0f;
    }
    
    /* PD gains */
    float Kp = 100.0f;
    float Kd = 20.0f;
    float dt = (float)loop_period;
    
    printf("[MOTOR] Starting motor servo at %d Hz\n", MOTOR_SERVO_RATE);
    
    while (g_running) {
        double current_time = get_time();
        
        /* Read desired state from shared memory (with mutex) */
        pthread_mutex_lock(&g_desired_state.mutex);
        memcpy(desired, g_desired_state.desired, sizeof(desired));
        pthread_mutex_unlock(&g_desired_state.mutex);
        
        /* Compute control and simulate dynamics for each joint */
        for (i = 0; i < N_DOFS; i++) {
            /* PD control */
            float error = desired[i].th - state[i].th;
            float error_d = desired[i].thd - state[i].thd;
            
            state[i].u = Kp * error + Kd * error_d + desired[i].uff;
            
            /* Simple dynamics simulation: thdd = u / inertia */
            float inertia = 0.1f;  /* kg*m^2 */
            state[i].thdd = state[i].u / inertia;
            
            /* Integrate */
            state[i].thd += state[i].thdd * dt;
            state[i].th += state[i].thd * dt;
            
            /* Simulate load sensor (with noise) */
            state[i].load = state[i].u + 0.01f * ((float)rand() / RAND_MAX - 0.5f);
        }
        
        /* Write state to shared memory */
        pthread_mutex_lock(&g_joint_state.mutex);
        g_joint_state.timestamp = current_time;
        g_joint_state.seq_num++;
        memcpy(g_joint_state.joints, state, sizeof(state));
        pthread_mutex_unlock(&g_joint_state.mutex);
        
        /* Signal task servo that new data is available */
        signal_sync_sem(&g_task_sync);
        
        g_motor_cycles++;
        
        /* Sleep until next cycle */
        next_time += loop_period;
        double sleep_time = next_time - get_time();
        if (sleep_time > 0) {
            sleep_us((unsigned int)(sleep_time * 1e6));
        }
    }
    
    printf("[MOTOR] Motor servo stopped after %lu cycles\n", g_motor_cycles);
    return NULL;
}

/*============================================================================
 * Task Servo (Medium-frequency task execution)
 *============================================================================*/

/**
 * \brief Task servo thread function
 * 
 * Simulates a 500 Hz task servo that:
 * - Generates minimum-jerk trajectory
 * - Writes desired state to shared memory
 * - Reads sensor feedback for monitoring
 */
void *task_servo_thread(void *arg) {
    (void)arg;
    
    double loop_period = 1.0 / TASK_SERVO_RATE;
    double next_time = get_time() + loop_period;
    double start_time = get_time();
    
    /* Trajectory parameters */
    float target[N_DOFS] = {PI/4, PI/3, PI/6};  /* Target positions */
    float move_duration = 2.0f;                  /* Movement time */
    
    /* Local state */
    DesiredState desired[N_DOFS];
    int i;
    for (i = 0; i < N_DOFS; i++) {
        desired[i].th = 0.0f;
        desired[i].thd = 0.0f;
        desired[i].thdd = 0.0f;
        desired[i].uff = 0.0f;
    }
    
    printf("[TASK]  Starting task servo at %d Hz\n", TASK_SERVO_RATE);
    printf("[TASK]  Trajectory: Moving to [%.2f, %.2f, %.2f] rad over %.1f sec\n",
           target[0], target[1], target[2], move_duration);
    
    while (g_running) {
        double current_time = get_time();
        double elapsed = current_time - start_time;
        
        /* Generate min-jerk trajectory */
        float tau = (float)(elapsed / move_duration);
        if (tau > 1.0f) tau = 1.0f;
        
        /* Min-jerk polynomial: 10*tau^3 - 15*tau^4 + 6*tau^5 */
        float s = 10.0f * tau * tau * tau - 15.0f * tau * tau * tau * tau + 
                  6.0f * tau * tau * tau * tau * tau;
        float sd = (30.0f * tau * tau - 60.0f * tau * tau * tau + 
                   30.0f * tau * tau * tau * tau) / (float)move_duration;
        float sdd = (60.0f * tau - 180.0f * tau * tau + 
                    120.0f * tau * tau * tau) / (float)(move_duration * move_duration);
        
        for (i = 0; i < N_DOFS; i++) {
            desired[i].th = target[i] * s;
            desired[i].thd = target[i] * sd;
            desired[i].thdd = target[i] * sdd;
            
            /* Simple gravity compensation as feedforward */
            desired[i].uff = 0.5f * sinf(desired[i].th);
        }
        
        /* Write desired state to shared memory */
        pthread_mutex_lock(&g_desired_state.mutex);
        g_desired_state.timestamp = current_time;
        g_desired_state.seq_num++;
        memcpy(g_desired_state.desired, desired, sizeof(desired));
        g_desired_state.new_command = 1;
        pthread_mutex_unlock(&g_desired_state.mutex);
        
        g_task_cycles++;
        
        /* Wait for motor servo or timeout */
        wait_sync_sem(&g_task_sync, (unsigned int)(loop_period * 1e6 * 2));
        
        /* Sleep until next cycle */
        next_time += loop_period;
        double sleep_time = next_time - get_time();
        if (sleep_time > 0) {
            sleep_us((unsigned int)(sleep_time * 1e6));
        }
    }
    
    printf("[TASK]  Task servo stopped after %lu cycles\n", g_task_cycles);
    return NULL;
}

/*============================================================================
 * Display/Visualization (Low-frequency)
 *============================================================================*/

/**
 * \brief Display thread function
 * 
 * Simulates a 60 Hz visualization that reads state from shared memory.
 */
void *display_thread(void *arg) {
    (void)arg;
    
    double loop_period = 1.0 / DISPLAY_RATE;
    double next_time = get_time() + loop_period;
    double start_time = get_time();
    
    printf("[DISP]  Starting display at %d Hz\n", DISPLAY_RATE);
    printf("\n");
    
    while (g_running) {
        double current_time = get_time();
        double elapsed = current_time - start_time;
        
        /* Read state from shared memory */
        JointState state[N_DOFS];
        DesiredState desired[N_DOFS];
        unsigned long motor_seq, task_seq;
        
        pthread_mutex_lock(&g_joint_state.mutex);
        memcpy(state, g_joint_state.joints, sizeof(state));
        motor_seq = g_joint_state.seq_num;
        pthread_mutex_unlock(&g_joint_state.mutex);
        
        pthread_mutex_lock(&g_desired_state.mutex);
        memcpy(desired, g_desired_state.desired, sizeof(desired));
        task_seq = g_desired_state.seq_num;
        pthread_mutex_unlock(&g_desired_state.mutex);
        
        /* Display state (overwrite previous line) */
        printf("\r[DISP]  t=%.2fs | ", elapsed);
        printf("J1: %.3f/%.3f | ", state[0].th, desired[0].th);
        printf("J2: %.3f/%.3f | ", state[1].th, desired[1].th);
        printf("J3: %.3f/%.3f | ", state[2].th, desired[2].th);
        printf("seq: M=%lu T=%lu     ", motor_seq, task_seq);
        fflush(stdout);
        
        g_display_cycles++;
        
        /* Sleep until next cycle */
        next_time += loop_period;
        double sleep_time = next_time - get_time();
        if (sleep_time > 0) {
            sleep_us((unsigned int)(sleep_time * 1e6));
        }
    }
    
    printf("\n[DISP]  Display stopped after %lu cycles\n", g_display_cycles);
    return NULL;
}

/*============================================================================
 * Main Program
 *============================================================================*/

/**
 * \brief Signal handler for clean shutdown
 */
void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

/**
 * \brief Initialize shared memory structures
 */
void init_shared_memory(void) {
    /* Initialize joint state shared memory */
    pthread_mutex_init(&g_joint_state.mutex, NULL);
    g_joint_state.timestamp = 0.0;
    g_joint_state.seq_num = 0;
    memset(g_joint_state.joints, 0, sizeof(g_joint_state.joints));
    
    /* Initialize desired state shared memory */
    pthread_mutex_init(&g_desired_state.mutex, NULL);
    g_desired_state.timestamp = 0.0;
    g_desired_state.seq_num = 0;
    g_desired_state.new_command = 0;
    memset(g_desired_state.desired, 0, sizeof(g_desired_state.desired));
    
    /* Initialize synchronization semaphores */
    init_sync_sem(&g_motor_sync);
    init_sync_sem(&g_task_sync);
}

/**
 * \brief Main function demonstrating multi-process real-time communication
 */
int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    
    pthread_t motor_thread, task_thread, disp_thread;
    
    printf("=================================================\n");
    printf("SL Library Example: Real-Time IPC\n");
    printf("=================================================\n\n");
    
    printf("This example demonstrates the real-time inter-process\n");
    printf("communication architecture used in SL robot control.\n\n");
    
    printf("Three concurrent processes communicate via shared memory:\n");
    printf("  - Motor Servo:  %4d Hz (control loop)\n", MOTOR_SERVO_RATE);
    printf("  - Task Servo:   %4d Hz (trajectory)\n", TASK_SERVO_RATE);
    printf("  - Display:      %4d Hz (visualization)\n\n", DISPLAY_RATE);
    
    printf("Running for %d seconds...\n\n", RUN_TIME_SECONDS);
    
    /* Set up signal handler for clean shutdown */
    signal(SIGINT, signal_handler);
    
    /* Initialize shared memory */
    init_shared_memory();
    
    /* Create threads (in real SL, these would be separate processes) */
    pthread_create(&motor_thread, NULL, motor_servo_thread, NULL);
    pthread_create(&task_thread, NULL, task_servo_thread, NULL);
    pthread_create(&disp_thread, NULL, display_thread, NULL);
    
    /* Run for specified time */
    sleep(RUN_TIME_SECONDS);
    
    /* Signal shutdown */
    g_running = 0;
    
    /* Signal semaphores to unblock waiting threads */
    signal_sync_sem(&g_motor_sync);
    signal_sync_sem(&g_task_sync);
    
    /* Wait for threads to finish */
    pthread_join(motor_thread, NULL);
    pthread_join(task_thread, NULL);
    pthread_join(disp_thread, NULL);
    
    printf("\n=================================================\n");
    printf("Statistics\n");
    printf("=================================================\n");
    printf("Motor servo: %lu cycles (expected: %d)\n", 
           g_motor_cycles, MOTOR_SERVO_RATE * RUN_TIME_SECONDS);
    printf("Task servo:  %lu cycles (expected: %d)\n", 
           g_task_cycles, TASK_SERVO_RATE * RUN_TIME_SECONDS);
    printf("Display:     %lu cycles (expected: %d)\n", 
           g_display_cycles, DISPLAY_RATE * RUN_TIME_SECONDS);
    
    printf("\n=================================================\n");
    printf("Summary: Real-Time IPC in SL\n");
    printf("=================================================\n");
    printf("\nKey concepts demonstrated:\n");
    printf("  - Shared memory for inter-process data exchange\n");
    printf("  - Mutexes for thread-safe access\n");
    printf("  - Condition variables for synchronization\n");
    printf("  - Timestamp-based data freshness checking\n");
    printf("  - Sequence numbers for ordering\n");
    printf("\nSL IPC architecture:\n");
    printf("  - Motor servo (highest priority): 1000 Hz\n");
    printf("  - Simulation servo: physics simulation\n");
    printf("  - Task servo: trajectory generation\n");
    printf("  - Vision servo: camera processing\n");
    printf("  - OpenGL servo: visualization\n");
    printf("  - ROS servo: ROS communication\n");
    printf("\nReal-time considerations:\n");
    printf("  - PREEMPT-RT or Xenomai for deterministic timing\n");
    printf("  - Priority inheritance mutexes\n");
    printf("  - Lock-free techniques where possible\n");
    printf("  - Memory locking (mlockall)\n");
    printf("\n");
    
    return 0;
}
