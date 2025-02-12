#ifndef __RINGBUFFER__
#define __RINGBUFFER__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct circular_buf_t circular_buf_t;


struct circular_buf_t {
    float *buffer;
    size_t head;
    size_t tail;
    size_t max;
    bool full; 

};

// Initializes the buffer. User provides the structure. This breaks encapsulation but avoids malloc. 
void circular_buffer_init(circular_buf_t* cbt, size_t size );

// Resets the buffer
void circular_buffer_reset(circular_buf_t* cbt);

// Adds new data to buffer, overwriting old if necessary
void circular_buffer_put(circular_buf_t* cbt, float data);

// Reads data from buffer into passed argument
int circular_buffer_get(circular_buf_t* cbt, float *data);

// Determines if buffer is full or not
bool circular_buffer_full(circular_buf_t* cbt);

// Determines if buffer is empty or not
bool circular_buffer_empty(circular_buf_t* cbt);

// Returns current size of buffer
size_t circular_buffer_size(circular_buf_t* cbt);

size_t circular_buffer_capacity(circular_buf_t* cbt);

// Sums up all the elements of the buffer (used for moving average implementations)
float circular_buffer_avg(circular_buf_t *cbt);



// Helper functions for write and read operations
static void advance_pointer(circular_buf_t* cbt);
static void retreat_pointer(circular_buf_t *cbt);

#endif