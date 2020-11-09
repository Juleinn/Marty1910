#ifndef AG1171_H_INCLUDED
#define AG1171_H_INCLUDED 

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

void ag1171_init();
void ag1171_ring_once();
void ag1171_ring_loop();

bool ag1171_is_offhook();

#endif
