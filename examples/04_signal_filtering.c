/*!=============================================================================
  ==============================================================================

  \file    04_signal_filtering.c

  \author  SL Examples
  \date    2024

  ==============================================================================
  \remarks

  Example demonstrating the Butterworth filter utilities in the SL library.
  
  The SL library provides 2nd order Butterworth low-pass filters for 
  smoothing sensor data. This is essential for:
  - Filtering noisy position and velocity measurements
  - Smoothing force/torque sensor readings
  - Processing vision data
  
  The filter implementation uses difference equations for real-time processing.

  Compile with:
    gcc -I$LAB_ROOT/include -I../include -L$LAB_LIBDIR -o 04_signal_filtering \
        04_signal_filtering.c -lSLcommon -lutility -lm

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

// SL specific headers
#include "SL.h"
#include "SL_filters.h"
#include "utility.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif

/**
 * \brief Initialize a filter structure
 * 
 * \param[out] f       Filter structure to initialize
 * \param[in]  cutoff  Cutoff frequency index (1-50, percentage of sample rate)
 */
void init_filter_struct(Filter *f, int cutoff) {
    f->cutoff = cutoff;
    f->raw[0] = f->raw[1] = f->raw[2] = 0.0;
    f->filt[0] = f->filt[1] = f->filt[2] = 0.0;
}

/**
 * \brief Generate a noisy sine wave signal
 * 
 * \param[in] t         Time value
 * \param[in] freq      Signal frequency (Hz)
 * \param[in] amplitude Amplitude of sine wave
 * \param[in] noise_amp Amplitude of noise
 * \return              Noisy signal value
 */
double noisy_sine(double t, double freq, double amplitude, double noise_amp) {
    double signal = amplitude * sin(2.0 * PI * freq * t);
    double noise = noise_amp * (2.0 * ((double)rand() / RAND_MAX) - 1.0);
    return signal + noise;
}

/**
 * \brief Main function demonstrating signal filtering
 */
int main(int argc, char **argv) {
    
    printf("=================================================\n");
    printf("SL Library Example: Signal Filtering\n");
    printf("=================================================\n");
    
    /* Initialize the filter system using SL library function */
    if (!init_filters()) {
        printf("Error: Could not initialize filters\n");
        printf("Note: init_filters() loads filter coefficients from files.\n");
        printf("      This example will demonstrate filter concepts.\n");
    }
    
    /* Seed random number generator for noise */
    srand(42);
    
    /* ----------------------------------------------------------------
     * Example 1: Butterworth Filter Basics
     * ----------------------------------------------------------------
     * Demonstrate the filter structure and parameters.
     */
    
    printf("\n--- Example 1: Filter Structure (SL_filters.h) ---\n");
    
    printf("\nSL Filter structure definition:\n");
    printf("  typedef struct Filter {\n");
    printf("    int cutoff;       /* Cutoff frequency index (1-%d) */\n", N_FILTERS);
    printf("    double raw[3];    /* Raw signal history */\n");
    printf("    double filt[3];   /* Filtered signal history */\n");
    printf("  } Filter;\n");
    
    printf("\nFilter constants from SL_filters.h:\n");
    printf("  FILTER_ORDER: %d (2nd order Butterworth)\n", FILTER_ORDER);
    printf("  N_FILTERS:    %d (number of available cutoff frequencies)\n", N_FILTERS);
    
    Filter position_filter;
    init_filter_struct(&position_filter, 10);  /* 10% cutoff */
    
    printf("\nExample filter configuration:\n");
    printf("  Filter order: 2 (2nd order Butterworth)\n");
    printf("  Cutoff index: %d (percentage of sampling rate)\n", position_filter.cutoff);
    printf("  History depth: 3 samples\n");
    
    /* ----------------------------------------------------------------
     * Example 2: Using the filt() Function
     * ----------------------------------------------------------------
     * Demonstrate the SL library's filt() function for filtering.
     */
    
    printf("\n--- Example 2: Using the SL filt() Function ---\n");
    
    printf("\nThe SL library provides the filt() function:\n");
    printf("  double filt(double raw, Filter *fptr);\n");
    printf("\nUsage:\n");
    printf("  Filter my_filter;\n");
    printf("  my_filter.cutoff = 10;  // Set cutoff index\n");
    printf("  // Initialize history to first sample or zero\n");
    printf("  filtered_value = filt(raw_value, &my_filter);\n");
    
    /* ----------------------------------------------------------------
     * Example 3: Filter Difference Equation
     * ----------------------------------------------------------------
     * Explain the 2nd order Butterworth filter implementation.
     */
    
    printf("\n--- Example 3: Filter Difference Equation ---\n");
    
    printf("\n2nd order Butterworth filter difference equation:\n");
    printf("  y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]\n");
    
    printf("\nWhere:\n");
    printf("  x[n]   = current raw sample\n");
    printf("  y[n]   = current filtered output\n");
    printf("  b0,b1,b2 = numerator coefficients (input)\n");
    printf("  a1,a2  = denominator coefficients (feedback)\n");
    
    printf("\nCoefficient loading:\n");
    printf("  - Coefficients stored in SensorFilters.cf file\n");
    printf("  - Loaded by init_filters() at startup\n");
    printf("  - Different coefficients for each cutoff index (1-%d)\n", N_FILTERS);
    
    /* ----------------------------------------------------------------
     * Example 4: Typical Filter Cutoff Selection
     * ----------------------------------------------------------------
     */
    
    printf("\n--- Example 4: Filter Cutoff Selection Guidelines ---\n");
    
    printf("\nTypical cutoff selections for robot sensors:\n");
    printf("  Signal Type       Cutoff    Reason\n");
    printf("  -----------------------------------------------\n");
    printf("  Joint position    5-10%%     Low noise, avoid phase lag\n");
    printf("  Joint velocity    10-20%%    More noise, need faster response\n");
    printf("  Force/torque      5-15%%     Balance noise vs. dynamics\n");
    printf("  Accelerometer     15-25%%    High freq noise, fast dynamics\n");
    printf("  Vision tracking   3-10%%     Smooth tracking\n");
    
    printf("\nTrade-offs:\n");
    printf("  Lower cutoff (e.g., 5%%):  More smoothing, more phase lag\n");
    printf("  Higher cutoff (e.g., 25%%): Less smoothing, less phase lag\n");
    
    /* ----------------------------------------------------------------
     * Example 5: Multi-Channel Filtering
     * ----------------------------------------------------------------
     * Show how to filter multiple sensor channels.
     */
    
    printf("\n--- Example 5: Multi-Channel Filtering in SL ---\n");
    
    int n_dofs = 7;  /* Example: 7-DOF robot arm */
    
    printf("\nFor a %d-DOF robot, you would have filter arrays:\n", n_dofs);
    printf("  Filter position_filters[%d+1];  // 1-indexed\n", n_dofs);
    printf("  Filter velocity_filters[%d+1];\n", n_dofs);
    printf("  Filter torque_filters[%d+1];\n", n_dofs);
    
    printf("\nInitialization (typically in sensor_proc):\n");
    printf("  for (i=1; i<=n_dofs; ++i) {\n");
    printf("    position_filters[i].cutoff = 10;  // or from config\n");
    printf("    velocity_filters[i].cutoff = 15;\n");
    printf("    torque_filters[i].cutoff = 8;\n");
    printf("  }\n");
    
    printf("\nUsage in sensor processing loop:\n");
    printf("  for (i=1; i<=n_dofs; ++i) {\n");
    printf("    joint_state[i].th = filt(raw_position[i], &position_filters[i]);\n");
    printf("    joint_state[i].thd = filt(raw_velocity[i], &velocity_filters[i]);\n");
    printf("    joint_state[i].load = filt(raw_torque[i], &torque_filters[i]);\n");
    printf("  }\n");
    
    /* ----------------------------------------------------------------
     * Example 6: Filter Response Characteristics
     * ----------------------------------------------------------------
     */
    
    printf("\n--- Example 6: Butterworth Filter Characteristics ---\n");
    
    printf("\nButterworth filter properties:\n");
    printf("  - Maximally flat passband (no ripple)\n");
    printf("  - Smooth roll-off in transition band\n");
    printf("  - -3dB at cutoff frequency\n");
    printf("  - -40dB/decade roll-off for 2nd order\n");
    
    printf("\nStep response:\n");
    printf("  - No overshoot (unlike Chebyshev)\n");
    printf("  - Smooth rise to final value\n");
    printf("  - Settling time depends on cutoff\n");
    
    printf("\nPhase lag:\n");
    printf("  - Lower cutoff = more phase lag\n");
    printf("  - Important for feedback control stability\n");
    printf("  - Consider predictive compensation if needed\n");
    
    /* ----------------------------------------------------------------
     * Example 7: Integration with SL Sensor Processing
     * ----------------------------------------------------------------
     */
    
    printf("\n--- Example 7: SL Sensor Processing Architecture ---\n");
    
    printf("\nSL sensor processing flow:\n");
    printf("  1. Raw sensor data arrives from hardware\n");
    printf("  2. Calibration applied (offsets, gains)\n");
    printf("  3. Filtering applied using filt()\n");
    printf("  4. Filtered data stored in joint_state[]\n");
    printf("  5. Controllers use filtered state data\n");
    
    printf("\nKey SL files for filtering:\n");
    printf("  - SL_filters.h: Filter structure definition\n");
    printf("  - SL_filters.c: filt() implementation\n");
    printf("  - SL_sensor_proc.c: Sensor processing pipeline\n");
    printf("  - config/SensorFilters.cf: Cutoff configurations\n");
    
    /* ----------------------------------------------------------------
     * Summary
     * ----------------------------------------------------------------
     */
    
    printf("\n=================================================\n");
    printf("Summary: Signal Filtering in SL\n");
    printf("=================================================\n");
    printf("\nKey concepts:\n");
    printf("  - 2nd order Butterworth low-pass filters\n");
    printf("  - Cutoff specified as percentage of sample rate\n");
    printf("  - Difference equation implementation\n");
    printf("\nSL filter usage:\n");
    printf("  1. Call init_filters() at startup\n");
    printf("  2. Declare Filter structure for each channel\n");
    printf("  3. Set cutoff index (1-%d)\n", N_FILTERS);
    printf("  4. Call filt(raw_value, &filter) each sample\n");
    printf("\nFilter selection guidelines:\n");
    printf("  - Position: 5-10%% cutoff\n");
    printf("  - Velocity: 10-20%% cutoff\n");
    printf("  - Force:    5-15%% cutoff\n");
    printf("  - Higher cutoff = faster response, more noise\n");
    printf("  - Lower cutoff = smoother output, more delay\n");
    printf("\n");
    
    return 0;
}
