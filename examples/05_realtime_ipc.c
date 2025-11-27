/*!=============================================================================
  ==============================================================================

  \file    05_realtime_ipc.c

  \author  SL Examples
  \date    2024

  ==============================================================================
  \remarks

  Example demonstrating real-time inter-process communication (IPC) as used
  in the SL library with PREEMPT-RT Linux or Xenomai.
  
  This example shows the SL servo architecture with three communicating
  processes:
  
  1. MOTOR SERVO: High-frequency control loop (typically 1000 Hz)
     - Reads joint commands from shared memory
     - Applies motor control and reads sensors
     - Writes sensor data to shared memory
  
  2. TASK SERVO: Medium-frequency task execution (typically 500 Hz)
     - Computes desired trajectories
     - Writes joint commands to shared memory
     - Reads sensor feedback
  
  3. OPENGL SERVO: Low-frequency visualization (60 Hz)
     - Reads state from shared memory
     - Displays robot state

  Key SL IPC concepts demonstrated:
  - Shared memory structures (smJointStates, smJointDesStates, etc.)
  - Semaphore synchronization
  - Message passing between servos
  - SL_rt_mutex for real-time safe synchronization

  Compile with:
    gcc -I$LAB_ROOT/include -I../include -L$LAB_LIBDIR -o 05_realtime_ipc \
        05_realtime_ipc.c -lSLcommon -lutility -lpthread -lm

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// SL specific headers
#include "SL.h"
#include "SL_shared_memory.h"
#include "SL_rt_mutex.h"
#include "utility.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif

/* Number of simulated DOFs for this example */
#define EXAMPLE_N_DOFS 3

/* Servo frequencies (Hz) - typical SL values */
#define MOTOR_SERVO_RATE   1000
#define TASK_SERVO_RATE    500
#define DISPLAY_RATE       60

/**
 * \brief Main function demonstrating SL IPC architecture
 */
int main(int argc, char **argv) {
    
    printf("=================================================\n");
    printf("SL Library Example: Real-Time IPC Architecture\n");
    printf("=================================================\n");
    
    /* ----------------------------------------------------------------
     * Example 1: SL Servo Architecture Overview
     * ----------------------------------------------------------------
     */
    
    printf("\n--- Example 1: SL Servo Architecture ---\n");
    
    printf("\nSL uses multiple concurrent processes (servos):\n");
    printf("\n  Servo Name          Typical Rate  Purpose\n");
    printf("  ----------------------------------------------------------\n");
    printf("  Motor Servo         %4d Hz      Low-level motor control\n", MOTOR_SERVO_RATE);
    printf("  Simulation Servo    %4d Hz      Physics simulation\n", MOTOR_SERVO_RATE);
    printf("  Task Servo          %4d Hz       Trajectory/task execution\n", TASK_SERVO_RATE);
    printf("  Vision Servo        %4d Hz       Camera processing\n", DISPLAY_RATE);
    printf("  OpenGL Servo        %4d Hz        3D visualization\n", DISPLAY_RATE);
    printf("  ROS Servo           Variable     ROS communication\n");
    
    /* ----------------------------------------------------------------
     * Example 2: Shared Memory Structures
     * ----------------------------------------------------------------
     */
    
    printf("\n--- Example 2: SL Shared Memory Structures ---\n");
    
    printf("\nKey shared memory structures (from SL_shared_memory.h):\n");
    
    printf("\n  smJointStates - Current joint state\n");
    printf("    - sm_sem: Semaphore for thread-safe access\n");
    printf("    - ts: Timestamp\n");
    printf("    - joint_state[]: Array of SL_fJstate (float version)\n");
    
    printf("\n  smJointDesStates - Desired joint state\n");
    printf("    - sm_sem: Semaphore\n");
    printf("    - ts: Timestamp\n");
    printf("    - joint_des_state[]: Array of SL_fDJstate\n");
    
    printf("\n  smBaseState - Floating base position\n");
    printf("    - sm_sem: Semaphore\n");
    printf("    - ts: Timestamp\n");
    printf("    - state[]: Array of SL_fCstate\n");
    
    printf("\n  smBaseOrient - Floating base orientation\n");
    printf("    - sm_sem: Semaphore\n");
    printf("    - ts: Timestamp\n");
    printf("    - orient[]: Array of SL_fquat\n");
    
    printf("\n  smMessage - Inter-servo messaging\n");
    printf("    - sm_sem: Semaphore\n");
    printf("    - n_msgs: Number of messages\n");
    printf("    - name[]: Message names\n");
    printf("    - buf[]: Message data buffer\n");
    
    /* ----------------------------------------------------------------
     * Example 3: Semaphore Synchronization
     * ----------------------------------------------------------------
     */
    
    printf("\n--- Example 3: SL Synchronization Semaphores ---\n");
    
    printf("\nSL uses semaphores for servo synchronization:\n");
    printf("  sm_motor_servo_sem        - Motor servo timing\n");
    printf("  sm_task_servo_sem         - Task servo timing\n");
    printf("  sm_simulation_servo_sem   - Simulation servo timing\n");
    printf("  sm_vision_servo_sem       - Vision servo timing\n");
    printf("  sm_openGL_servo_sem       - OpenGL servo timing\n");
    printf("  sm_ros_servo_sem          - ROS servo timing\n");
    
    printf("\nData-ready semaphores:\n");
    printf("  sm_joint_des_state_ready_sem - New desired state available\n");
    printf("  sm_raw_blobs_ready_sem       - New vision blobs available\n");
    printf("  sm_user_graphics_ready_sem   - User graphics data ready\n");
    
    printf("\nMessage-ready semaphores:\n");
    printf("  sm_task_message_ready_sem        - Message for task servo\n");
    printf("  sm_motor_message_ready_sem       - Message for motor servo\n");
    printf("  sm_simulation_message_ready_sem  - Message for sim servo\n");
    printf("  sm_openGL_message_ready_sem      - Message for openGL servo\n");
    
    /* ----------------------------------------------------------------
     * Example 4: Real-Time Mutex (SL_rt_mutex)
     * ----------------------------------------------------------------
     */
    
    printf("\n--- Example 4: SL Real-Time Mutex (SL_rt_mutex.h) ---\n");
    
    printf("\nSL provides cross-platform RT mutex wrappers:\n");
    printf("  - Uses Xenomai mutexes on PREEMPT-RT/Xenomai\n");
    printf("  - Falls back to pthread mutexes otherwise\n");
    
    printf("\nMutex functions:\n");
    printf("  sl_rt_mutex_init(sl_rt_mutex* mutex)\n");
    printf("  sl_rt_mutex_lock(sl_rt_mutex* mutex)\n");
    printf("  sl_rt_mutex_trylock(sl_rt_mutex* mutex)\n");
    printf("  sl_rt_mutex_unlock(sl_rt_mutex* mutex)\n");
    printf("  sl_rt_mutex_destroy(sl_rt_mutex* mutex)\n");
    
    printf("\nCondition variable functions:\n");
    printf("  sl_rt_cond_init(sl_rt_cond* cond)\n");
    printf("  sl_rt_cond_wait(sl_rt_cond* cond, sl_rt_mutex* mutex)\n");
    printf("  sl_rt_cond_timedwait(sl_rt_cond* cond, sl_rt_mutex* mutex, timeout)\n");
    printf("  sl_rt_cond_signal(sl_rt_cond* cond)\n");
    printf("  sl_rt_cond_broadcast(sl_rt_cond* cond)\n");
    
    /* ----------------------------------------------------------------
     * Example 5: Message Passing
     * ----------------------------------------------------------------
     */
    
    printf("\n--- Example 5: Inter-Servo Message Passing ---\n");
    
    printf("\nSL provides message passing functions:\n");
    printf("  sendMessageTaskServo(char *message, void *buf, int n_bytes)\n");
    printf("  sendMessageMotorServo(char *message, void *buf, int n_bytes)\n");
    printf("  sendMessageSimulationServo(char *message, void *buf, int n_bytes)\n");
    printf("  sendMessageOpenGLServo(char *message, void *buf, int n_bytes)\n");
    printf("  sendMessageVisionServo(char *message, void *buf, int n_bytes)\n");
    printf("  sendMessageROSServo(char *message, void *buf, int n_bytes)\n");
    
    printf("\nUsage example (from task servo to motor servo):\n");
    printf("  char msg_name[] = \"setGains\";\n");
    printf("  double gains[3] = {100.0, 20.0, 0.1};\n");
    printf("  sendMessageMotorServo(msg_name, gains, sizeof(gains));\n");
    
    printf("\nReceiving messages (in servo loop):\n");
    printf("  if (semTake(sm_task_message_ready_sem, NO_WAIT) == OK) {\n");
    printf("    // Process messages in sm_task_message\n");
    printf("    // Match message name and handle accordingly\n");
    printf("  }\n");
    
    /* ----------------------------------------------------------------
     * Example 6: Typical Data Flow
     * ----------------------------------------------------------------
     */
    
    printf("\n--- Example 6: SL Data Flow Between Servos ---\n");
    
    printf("\nTypical control loop data flow:\n");
    printf("\n  1. Motor Servo (or Simulation Servo):\n");
    printf("     - Reads joint_des_state from shared memory\n");
    printf("     - Computes motor commands (or physics sim)\n");
    printf("     - Writes joint_state to shared memory\n");
    printf("     - Signals sm_task_servo_sem\n");
    
    printf("\n  2. Task Servo:\n");
    printf("     - Waits on sm_task_servo_sem\n");
    printf("     - Reads joint_state from shared memory\n");
    printf("     - Runs task/trajectory computation\n");
    printf("     - Writes joint_des_state to shared memory\n");
    printf("     - Signals sm_joint_des_state_ready_sem\n");
    
    printf("\n  3. OpenGL Servo:\n");
    printf("     - Waits on sm_openGL_servo_sem (60 Hz trigger)\n");
    printf("     - Reads joint_state from shared memory\n");
    printf("     - Updates 3D visualization\n");
    
    /* ----------------------------------------------------------------
     * Example 7: Shared Memory Initialization
     * ----------------------------------------------------------------
     */
    
    printf("\n--- Example 7: Shared Memory Initialization ---\n");
    
    printf("\nSL shared memory is initialized with init_shared_memory():\n");
    printf("  - Called during SL startup\n");
    printf("  - Creates all shared memory segments\n");
    printf("  - Initializes all semaphores\n");
    printf("  - Returns TRUE on success\n");
    
    printf("\nInitialization sequence:\n");
    printf("  1. init_shared_memory() - Create SM and semaphores\n");
    printf("  2. Start motor/simulation servo\n");
    printf("  3. Start task servo\n");
    printf("  4. Start openGL servo\n");
    printf("  5. Start additional servos as needed\n");
    
    printf("\nKey shared memory objects created:\n");
    printf("  - sm_joint_state (smJointStates)\n");
    printf("  - sm_joint_des_state (smJointDesStates)\n");
    printf("  - sm_joint_sim_state (smJointSimStates)\n");
    printf("  - sm_base_state (smBaseState)\n");
    printf("  - sm_base_orient (smBaseOrient)\n");
    printf("  - sm_cart_states (smCartStates)\n");
    printf("  - sm_misc_sensor (smMiscSensors)\n");
    printf("  - sm_contacts (smContacts)\n");
    printf("  - sm_user_graphics (smUserGraphics)\n");
    printf("  - sm_oscilloscope (smOscilloscope)\n");
    
    /* ----------------------------------------------------------------
     * Example 8: Real-Time Considerations
     * ----------------------------------------------------------------
     */
    
    printf("\n--- Example 8: Real-Time Considerations ---\n");
    
    printf("\nFor deterministic real-time performance:\n");
    
    printf("\n  PREEMPT-RT or Xenomai kernel:\n");
    printf("    - Provides deterministic scheduling\n");
    printf("    - Sub-millisecond latency\n");
    printf("    - Priority inheritance for mutexes\n");
    
    printf("\n  Memory locking:\n");
    printf("    - mlockall(MCL_CURRENT | MCL_FUTURE)\n");
    printf("    - Prevents page faults during RT operation\n");
    
    printf("\n  Thread priorities (SCHED_FIFO):\n");
    printf("    - Motor servo: Highest priority\n");
    printf("    - Task servo: High priority\n");
    printf("    - Vision servo: Medium priority\n");
    printf("    - OpenGL servo: Lower priority\n");
    
    printf("\n  Lock-free techniques:\n");
    printf("    - Double buffering where possible\n");
    printf("    - Atomic operations for flags\n");
    printf("    - Minimize critical section duration\n");
    
    /* ----------------------------------------------------------------
     * Summary
     * ----------------------------------------------------------------
     */
    
    printf("\n=================================================\n");
    printf("Summary: SL Real-Time IPC Architecture\n");
    printf("=================================================\n");
    printf("\nKey concepts:\n");
    printf("  - Multiple concurrent servo processes\n");
    printf("  - Shared memory for data exchange (SL_shared_memory.h)\n");
    printf("  - Semaphores for synchronization\n");
    printf("  - Message passing for commands\n");
    printf("  - RT mutex wrappers (SL_rt_mutex.h)\n");
    printf("\nSL servo architecture:\n");
    printf("  - Motor servo (1000 Hz): Hardware interface\n");
    printf("  - Simulation servo: Physics simulation\n");
    printf("  - Task servo (500 Hz): Trajectory/control\n");
    printf("  - Vision servo: Camera processing\n");
    printf("  - OpenGL servo (60 Hz): Visualization\n");
    printf("  - ROS servo: ROS communication\n");
    printf("\nReal-time considerations:\n");
    printf("  - Use PREEMPT-RT or Xenomai\n");
    printf("  - Lock memory with mlockall()\n");
    printf("  - Set appropriate thread priorities\n");
    printf("  - Minimize blocking in RT threads\n");
    printf("\n");
    
    return 0;
}
