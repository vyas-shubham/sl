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

  This is a standalone example that can be compiled without external dependencies.

  ============================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

/**
 * \brief Filter structure (as defined in SL_filters.h)
 * 
 * Holds the state for a 2nd order Butterworth filter.
 */
typedef struct {
  int cutoff;        /**< Cutoff frequency index */
  double raw[3];     /**< Raw signal history */
  double filt[3];    /**< Filtered signal history */
} Filter;

/**
 * \brief Initialize a filter structure
 * 
 * \param[out] f       Filter structure to initialize
 * \param[in]  cutoff  Cutoff frequency index (1-50, percentage of sample rate)
 */
void init_filter(Filter *f, int cutoff) {
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
 * \brief Simple 2nd order Butterworth filter implementation
 * 
 * This demonstrates the filter algorithm used in SL.
 * In practice, you would use the filt() function from SL_filters.c
 * 
 * \param[in]     raw       Raw input value
 * \param[in,out] filter    Filter structure with history
 * \param[in]     a         Filter coefficients a[0..2]
 * \param[in]     b         Filter coefficients b[0..2]
 * \return                  Filtered output value
 */
double butterworth_filter(double raw, Filter *filter, double *a, double *b) {
    /* Shift history */
    filter->raw[2] = filter->raw[1];
    filter->raw[1] = filter->raw[0];
    filter->raw[0] = raw;
    
    filter->filt[2] = filter->filt[1];
    filter->filt[1] = filter->filt[0];
    
    /* Compute filtered output using difference equation */
    filter->filt[0] = b[0] * filter->raw[0] +
                      b[1] * filter->raw[1] +
                      b[2] * filter->raw[2] -
                      a[1] * filter->filt[1] -
                      a[2] * filter->filt[2];
    
    return filter->filt[0];
}

/**
 * \brief Main function demonstrating signal filtering
 */
int main(int argc, char **argv) {
    
    printf("=================================================\n");
    printf("SL Library Example: Signal Filtering\n");
    printf("=================================================\n");
    
    /* Seed random number generator for noise */
    srand(42);
    
    /* ----------------------------------------------------------------
     * Example 1: Butterworth Filter Basics
     * ----------------------------------------------------------------
     * Demonstrate the filter structure and parameters.
     */
    
    printf("\n--- Example 1: Filter Structure ---\n");
    
    Filter position_filter;
    init_filter(&position_filter, 10);  /* 10% cutoff */
    
    printf("\nFilter structure (SL_filters.h):\n");
    printf("  typedef struct Filter {\n");
    printf("    int cutoff;       /* Cutoff frequency index */\n");
    printf("    double raw[3];    /* Raw signal history */\n");
    printf("    double filt[3];   /* Filtered signal history */\n");
    printf("  } Filter;\n");
    
    printf("\nFilter parameters:\n");
    printf("  Filter order: 2 (2nd order Butterworth)\n");
    printf("  Cutoff index: %d (percentage of sampling rate)\n", position_filter.cutoff);
    printf("  History depth: 3 samples\n");
    
    /* ----------------------------------------------------------------
     * Example 2: Filter Coefficients
     * ----------------------------------------------------------------
     * Show typical Butterworth filter coefficients.
     */
    
    printf("\n--- Example 2: Filter Coefficients ---\n");
    
    /* Example coefficients for a 10% cutoff 2nd order Butterworth filter */
    /* These would typically be loaded from a file in SL */
    double a_10[3] = {1.0, -1.561, 0.6414};   /* Denominator coefficients */
    double b_10[3] = {0.0201, 0.0402, 0.0201}; /* Numerator coefficients */
    
    printf("\nExample coefficients (10%% cutoff):\n");
    printf("  a[0]=%7.4f  a[1]=%7.4f  a[2]=%7.4f\n", a_10[0], a_10[1], a_10[2]);
    printf("  b[0]=%7.4f  b[1]=%7.4f  b[2]=%7.4f\n", b_10[0], b_10[1], b_10[2]);
    
    printf("\nDifference equation:\n");
    printf("  y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]\n");
    
    /* ----------------------------------------------------------------
     * Example 3: Filtering a Noisy Signal
     * ----------------------------------------------------------------
     * Apply filter to a noisy sine wave.
     */
    
    printf("\n--- Example 3: Filtering Noisy Signal ---\n");
    
    double sample_rate = 100.0;  /* 100 Hz */
    double signal_freq = 2.0;    /* 2 Hz signal */
    double amplitude = 1.0;
    double noise_amp = 0.3;      /* 30% noise */
    double duration = 1.0;       /* 1 second */
    double dt = 1.0 / sample_rate;
    
    printf("\nSignal parameters:\n");
    printf("  Sample rate: %.0f Hz\n", sample_rate);
    printf("  Signal frequency: %.1f Hz\n", signal_freq);
    printf("  Signal amplitude: %.1f\n", amplitude);
    printf("  Noise amplitude: %.1f (%.0f%%)\n", noise_amp, 100 * noise_amp / amplitude);
    
    /* Initialize filter */
    Filter signal_filter;
    init_filter(&signal_filter, 10);
    
    printf("\nSample filtered output (every 0.05s):\n");
    printf("  Time      Raw        Filtered   True Signal\n");
    printf("  -------------------------------------------\n");
    
    double t;
    int sample_count = 0;
    int print_interval = 5;  /* Print every 5 samples */
    double error_sum_raw = 0.0;
    double error_sum_filt = 0.0;
    
    for (t = 0.0; t <= duration; t += dt) {
        double true_signal = amplitude * sin(2.0 * PI * signal_freq * t);
        double noisy_signal = noisy_sine(t, signal_freq, amplitude, noise_amp);
        double filtered = butterworth_filter(noisy_signal, &signal_filter, a_10, b_10);
        
        /* Accumulate errors */
        error_sum_raw += (noisy_signal - true_signal) * (noisy_signal - true_signal);
        error_sum_filt += (filtered - true_signal) * (filtered - true_signal);
        
        if (sample_count % print_interval == 0) {
            printf("  %5.2f s   %8.4f   %8.4f   %8.4f\n", 
                   t, noisy_signal, filtered, true_signal);
        }
        sample_count++;
    }
    
    /* Compute RMS errors */
    double rms_raw = sqrt(error_sum_raw / sample_count);
    double rms_filt = sqrt(error_sum_filt / sample_count);
    
    printf("\nFiltering performance:\n");
    printf("  RMS error (raw signal):      %.4f\n", rms_raw);
    printf("  RMS error (filtered signal): %.4f\n", rms_filt);
    printf("  Improvement: %.1f%%\n", 100.0 * (rms_raw - rms_filt) / rms_raw);
    
    /* ----------------------------------------------------------------
     * Example 4: Effect of Cutoff Frequency
     * ----------------------------------------------------------------
     * Compare different cutoff frequencies.
     */
    
    printf("\n--- Example 4: Cutoff Frequency Comparison ---\n");
    
    /* Coefficients for different cutoffs (simplified examples) */
    double a_5[3] = {1.0, -1.778, 0.8008};   /* 5% cutoff - more aggressive */
    double b_5[3] = {0.0056, 0.0111, 0.0056};
    
    double a_20[3] = {1.0, -1.142, 0.4128};  /* 20% cutoff - less aggressive */
    double b_20[3] = {0.0675, 0.1350, 0.0675};
    
    /* Initialize filters */
    Filter filter_5, filter_10_cmp, filter_20;
    init_filter(&filter_5, 5);
    init_filter(&filter_10_cmp, 10);
    init_filter(&filter_20, 20);
    
    printf("\nComparing filter cutoffs (5%%, 10%%, 20%%):\n");
    printf("  Sample    Raw       5%%        10%%       20%%       True\n");
    printf("  --------------------------------------------------------\n");
    
    srand(42);  /* Reset for reproducibility */
    sample_count = 0;
    
    for (t = 0.0; t <= 0.5; t += dt) {  /* 0.5 second comparison */
        double true_signal = amplitude * sin(2.0 * PI * signal_freq * t);
        double noisy_signal = noisy_sine(t, signal_freq, amplitude, noise_amp);
        
        double filt_5 = butterworth_filter(noisy_signal, &filter_5, a_5, b_5);
        double filt_10 = butterworth_filter(noisy_signal, &filter_10_cmp, a_10, b_10);
        double filt_20 = butterworth_filter(noisy_signal, &filter_20, a_20, b_20);
        
        if (sample_count % 5 == 0) {
            printf("  %3d      %7.3f   %7.3f   %7.3f   %7.3f   %7.3f\n",
                   sample_count, noisy_signal, filt_5, filt_10, filt_20, true_signal);
        }
        sample_count++;
    }
    
    printf("\nCutoff frequency trade-offs:\n");
    printf("  Lower cutoff (5%%):  More smoothing, more phase lag\n");
    printf("  Higher cutoff (20%%): Less smoothing, less phase lag\n");
    
    /* ----------------------------------------------------------------
     * Example 5: Multiple Signal Filtering
     * ----------------------------------------------------------------
     * Filter multiple sensor channels simultaneously.
     */
    
    printf("\n--- Example 5: Multi-Channel Filtering ---\n");
    
    int n_channels = 3;
    char *channel_names[] = {"Position", "Velocity", "Force"};
    Filter channels[3];
    double raw_values[3];
    double filtered_values[3];
    int cutoffs[] = {10, 15, 5};  /* Different cutoffs per channel */
    
    /* Initialize filters */
    int i;
    for (i = 0; i < n_channels; i++) {
        init_filter(&channels[i], cutoffs[i]);
    }
    
    printf("\nMulti-channel filter configuration:\n");
    for (i = 0; i < n_channels; i++) {
        printf("  %s: cutoff=%d%%\n", channel_names[i], cutoffs[i]);
    }
    
    printf("\nSample multi-channel output:\n");
    printf("  Time    Raw(pos,vel,force)       Filtered(pos,vel,force)\n");
    printf("  --------------------------------------------------------\n");
    
    srand(42);
    sample_count = 0;
    
    for (t = 0.0; t <= 0.3; t += dt) {
        /* Generate noisy signals for each channel */
        raw_values[0] = noisy_sine(t, 1.0, 1.0, 0.2);   /* Position */
        raw_values[1] = noisy_sine(t, 2.0, 0.5, 0.15);  /* Velocity */
        raw_values[2] = noisy_sine(t, 0.5, 10.0, 3.0);  /* Force */
        
        /* Filter each channel */
        filtered_values[0] = butterworth_filter(raw_values[0], &channels[0], a_10, b_10);
        filtered_values[1] = butterworth_filter(raw_values[1], &channels[1], a_20, b_20);
        filtered_values[2] = butterworth_filter(raw_values[2], &channels[2], a_5, b_5);
        
        if (sample_count % 3 == 0) {
            printf("  %5.2f   (%6.2f,%5.2f,%6.1f)   (%6.2f,%5.2f,%6.1f)\n",
                   t,
                   raw_values[0], raw_values[1], raw_values[2],
                   filtered_values[0], filtered_values[1], filtered_values[2]);
        }
        sample_count++;
    }
    
    /* ----------------------------------------------------------------
     * Example 6: Step Response
     * ----------------------------------------------------------------
     * Observe filter response to a step input.
     */
    
    printf("\n--- Example 6: Step Response ---\n");
    
    Filter step_filter;
    init_filter(&step_filter, 10);
    
    printf("\nFilter step response (10%% cutoff):\n");
    printf("  Sample    Input    Output\n");
    printf("  --------------------------\n");
    
    for (i = 0; i < 30; i++) {
        double input = (i >= 5) ? 1.0 : 0.0;  /* Step at sample 5 */
        double output = butterworth_filter(input, &step_filter, a_10, b_10);
        printf("  %3d       %.1f      %.4f\n", i, input, output);
    }
    
    printf("\nObservations:\n");
    printf("  - Filter has delay (phase lag)\n");
    printf("  - Output smoothly approaches input\n");
    printf("  - No overshoot (Butterworth characteristic)\n");
    
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
    printf("  1. Declare Filter structure\n");
    printf("  2. Initialize with cutoff frequency\n");
    printf("  3. Call filt(raw_value, &filter) each sample\n");
    printf("\nFilter selection guidelines:\n");
    printf("  - Position: 5-10%% cutoff\n");
    printf("  - Velocity: 10-20%% cutoff\n");
    printf("  - Force:    5-15%% cutoff\n");
    printf("  - Higher cutoff = faster response, more noise\n");
    printf("  - Lower cutoff = smoother output, more delay\n");
    printf("\n");
    
    return 0;
}
