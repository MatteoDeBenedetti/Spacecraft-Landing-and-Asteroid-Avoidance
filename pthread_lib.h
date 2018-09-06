/*----------------------------------------------------------------------------*/
// PTHREAD LIBRARY --- HEADER
// Useful functions for handling threads
/*----------------------------------------------------------------------------*/

#ifndef PTHREAD_LIB_H
#define PTHREAD_LIB_H

/*----------------------------------------------------------------------------*/


#include <time.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>


/*----------------------------------------------------------------------------*/
/*Constants------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
/*Structures------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

struct task_param 
// structure of parameters to be passed to the thread task. 
// always set the first 5
{	
	int arg; // argument
	int period; // period in ms
	int deadline; // relative deadline in ms
	int priority; // priority 1-99
	int missCount; // counter of dl missed
	struct timespec at; //online next activ time
	struct timespec dl; //online abs deadline
};

/*----------------------------------------------------------------------------*/
/*Functions Prototipes--------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void time_copy(struct timespec *td, struct timespec ts);
void time_add_ms(struct timespec *t, int ms);
int time_cmp(struct timespec t1, struct timespec t2);
void set_period(struct task_param *tp);
int deadline_miss(struct task_param *tp);
int task_init(struct task_param tp, struct sched_param sp, pthread_attr_t att, pthread_t tid, int sched, void* task);
int task_init2(struct task_param *tp, struct sched_param *sp, pthread_attr_t att, pthread_t *tid, int sched, void* task);
void create_mutex(pthread_mutex_t mux, pthread_mutexattr_t mux_attr, int prio, int ceil);
int sign(float x);
float min(float a, float b);
float max(float a, float b);
float randf(float min, float max);
int is_close(float a, float b, float tol);





/*----------------------------------------------------------------------------*/

#endif

