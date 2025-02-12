#include <app/circularbuffer.h>
//#include <assert.h>


void circular_buffer_init(circular_buf_t* cbt, size_t size)
{
    //assert(cbt && size);
    cbt->max = size; 
    circular_buffer_reset(cbt);
    return;

}

void circular_buffer_reset(circular_buf_t* cbt)
{
    //assert(cbt);
    cbt->head = 0;
    cbt->tail = 0;
    cbt->full = false;
    return;
}

void circular_buffer_put(circular_buf_t* cbt, float data)
{

    //assert(cbt && cbt->buffer);

    cbt->buffer[cbt->head] = data;

    advance_pointer(cbt);

    return;
}

int circular_buffer_get(circular_buf_t* cbt, float *data)
{
    //assert(cbt && cbt->buffer && data);

    int r = -1;

    if(!circular_buffer_empty(cbt))
    {
        *data = cbt->buffer[cbt->tail];
        retreat_pointer(cbt);

        r = 0;
    }
    return r;
}

float circular_buffer_avg(circular_buf_t *cbt)
{
    float sum = 0;
    //assert(cbt && cbt->buffer);
    size_t size = circular_buffer_size(cbt);
    for(int i = 0; i < size; i++)
    {
        sum += cbt->buffer[i];
    }
    return sum/size;
}

bool circular_buffer_full(circular_buf_t* cbt)
{
    //assert(cbt);
    return cbt->full;
}

bool circular_buffer_empty(circular_buf_t* cbt)
{
    //assert(cbt);
    return (!(cbt->full) && (cbt->head == cbt->tail));
}

size_t circular_buffer_capacity(circular_buf_t* cbt)
{
    //assert(cbt);
    return cbt->max;
}

size_t circular_buffer_size(circular_buf_t* cbt)
{
    //assert(cbt);

    size_t size = cbt->max;

    if (cbt->full)
    {
        return size;
    }
    
    if (cbt->head > cbt->tail)
    {
        size = (cbt->head - cbt->tail);
        return size;
    }
    
    size = (cbt->max + cbt->head - cbt->tail);
    return size;
}


static void advance_pointer(circular_buf_t *cbt)
{
    //assert(cbt);

    if(cbt->full)
    {
        if (++(cbt->tail) == cbt->max)
        {
            cbt->tail = 0;
        }
    }

    if (++(cbt->head) == cbt->max)
    {
        cbt->head = 0;
    }

    // If the next write location is also the last write location, the buffer is full
    cbt->full = (cbt->head == cbt->tail);
}

static void retreat_pointer(circular_buf_t *cbt)
{
    //assert(cbt);

    // Removing an element means the buffer is now not full
    cbt->full = false;

    // When reading the last location (tail), the tail needs to increment with wraparound. 
    if(++(cbt->tail) == cbt->max)
    {
        cbt->tail = 0;
    }
}