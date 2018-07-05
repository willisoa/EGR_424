/*
 * SoundVisualizer.c
 *
 *  Created on: Jul 3, 2018
 *      Author: Zach Ash
 *      Alec Willison
 */
#include "SoundVisualizer.h"

// The number of readings in a sample for 200ms
#define BUFFER_SIZE 3200
// The number of recent samples to keep
#define NUMBER_BUFFERS 2

// Indicates if a sample is ready to process
static int sample_ready = 0;
// The current buffer being written to
static int current_buffer = 0;
// The current reading in the current buffer
static int current_reading_index = 0;
// The data buffers for each sample
static int sample_buffers[NUMBER_BUFFERS][BUFFER_SIZE];

/************************************************************
 * is_sample_ready: Returns whether a sample is ready for
 * further processing or not.
 ************************************************************/
int is_sample_ready()
{
    return sample_ready;
}

/************************************************************
 * average: Computes the average reading from the ADC over
 * the past 200ms. Uses basic averaging code.
 ************************************************************/
int average(int* values, int num_values) {
    int sum = 0, i;
    for(i = 0; i < num_values; i++) {
        sum += values[i];
    }
    return sum / num_values;
}

/************************************************************
 * handle_sample: Computes the average ADC value over the
 * past 200ms and resets the global sample_ready variable to
 * not ready.
 ************************************************************/
int handle_sample() {
    // Compute the average of the sample's readings
    int last_buffer = (current_buffer + NUMBER_BUFFERS - 1) % NUMBER_BUFFERS;
    int avg = average(sample_buffers[last_buffer], BUFFER_SIZE);
    sample_ready = 0;
    return avg;
}

/************************************************************
 * save_reading: Stores an ADC reading in the correct buffer
 * at the correct index. If the buffer size reaches 200ms,
 * then a sample is ready.
 ************************************************************/
void save_reading(int reading) {
    // Add the reading to the buffer
    sample_buffers[current_buffer][current_reading_index] = reading;

    // Update the reading index
    current_reading_index++;

    // If a buffer is finished, move to the next buffer
    if(current_reading_index >= BUFFER_SIZE) {
        current_reading_index = 0;
        current_buffer = (current_buffer + 1) % NUMBER_BUFFERS;

        // Mark the sample as ready
        sample_ready = 1;
    }
}
