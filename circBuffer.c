#include "circBuffer.h"

char char_dequeue(volatile circBuffer *buffP)
{

    if (buffP->size > 0)
    {
        buffP->size--;
//important to mod before it's used (even though with post-incriment the value will go 1 above, it won't be used before being mod'd)
		buffP->tail %= BUFSIZE; 
		return buffP->buf[buffP->tail++];
    }
    return 0x00;
}

int char_queue(char c, volatile circBuffer *buffP)
{
	// Buffer char if we have room
	if (buffP->size < BUFSIZE)
	{
		buffP->size++;
		buffP->head %= BUFSIZE;
		buffP->buf[buffP->head++] = c;
		return 1;
	}
	else
	{
		// Character is lost due to full buffer
		return 0;
	}
}
int char_avail(volatile circBuffer *buffP) //Is there something waiting in the queue?
{
    return buffP->size;
}
