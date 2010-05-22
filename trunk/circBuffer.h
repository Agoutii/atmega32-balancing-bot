#ifndef CIRCBUFFER_H
#define CIRCBUFFER_H

#include <stdio.h>
#include <ctype.h> 
#ifndef BUFSIZE
  #define BUFSIZE 90
#endif
typedef struct
{
	unsigned int head, tail, size; 
 	//N.B. Head points to the next empty space, tail to the next char to be pulled out.
	char buf[BUFSIZE];
} circBuffer;

int char_avail(volatile circBuffer *buffP);
char char_dequeue(volatile circBuffer *buffP);
int char_queue(char c, volatile circBuffer *buffP);

#endif
