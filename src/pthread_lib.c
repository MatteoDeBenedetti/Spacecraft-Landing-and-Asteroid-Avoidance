/*----------------------------------------------------------------------------*/
// PTHREAD LIBRARY --- SOURCE
// Useful functions for handling threads
/*----------------------------------------------------------------------------*/

#define _GNU_SOURCE
#include <time.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <allegro.h>

#include "pthread_lib.h"


/*----------------------------------------------------------------------------*/
/*Functions Definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void time_copy(struct timespec *td, struct timespec ts)
// copies the time in arg2 to arg1
{
	td->tv_sec = ts.tv_sec;
	td->tv_nsec = ts.tv_nsec;
}

void time_add_ms(struct timespec *t, int ms)
// adds arg2 to arg1
{
	t->tv_sec += ms/1000; //converts and adds ms
	t->tv_nsec += (ms%1000)*1000000; //converts and adds ns
	
	// if ns > 1 s then add 1 second
	if (t->tv_nsec > 1000000000)
	{
		t->tv_nsec -= 1000000000;
		t->tv_sec += 1;
	}
}

int time_cmp(struct timespec t1, struct timespec t2)
// compares arg1 and arg2
{
	if (t1.tv_sec > t2.tv_sec) return 1;
	if (t1.tv_sec < t2.tv_sec) return -1;
	if (t1.tv_nsec > t2.tv_nsec) return 1;
	if (t1.tv_nsec > t2.tv_nsec) return -1;
	
	return 0;
}

void set_period(struct task_param *tp)
// set next activation time and absolute deadline of arg1
{
	struct timespec t;
	
	clock_gettime(CLOCK_MONOTONIC, &t);
	
	time_copy(&(tp->at), t);
	time_copy(&(tp->dl), t);
	time_add_ms(&(tp->at), tp->period);
	time_add_ms(&(tp->dl), tp->deadline);
}

void wait_for_period(struct task_param *tp)
// suspend until next activation time and update next at and dl
{
	// suspend until next activation time
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(tp->at), NULL);
	
	// update next at and dl
	time_add_ms(&(tp->at), tp->period);
	time_add_ms(&(tp->dl), tp->period);
}

int deadline_miss(struct task_param *tp)
// check if the task just missed its deadline and update total in tp.missCount
{
	struct	timespec	now;
	
	clock_gettime(CLOCK_MONOTONIC, &now);
	
	if (time_cmp(now, tp->dl) > 0)
	{
		tp->missCount ++;
		return 1;
	}
	
	return 0;
}

int task_init(struct task_param tp, struct sched_param sp, pthread_attr_t att, pthread_t tid, int sched, void* task)
// initialize and create a thread with: arg1=task_param, arg2=sched_param, arg3=pthread_attr, arg4=pthread_t, arg5=scheduler (1=FIFO, 2=RR, 3=DEADLINE, anything else=OTHER)
{
	int err;
	
	pthread_attr_init(&att); // initialize parameters 
		
	switch (sched)
	{
		case 1:
			pthread_attr_setinheritsched(&att, PTHREAD_EXPLICIT_SCHED); // set other sched
			pthread_attr_setschedpolicy(&att, SCHED_FIFO); // specify which sched
			break;
		case 2:
			pthread_attr_setinheritsched(&att, PTHREAD_EXPLICIT_SCHED); // set other sched
			pthread_attr_setschedpolicy(&att, SCHED_RR); // specify which sched
			break;
		case 3:
			pthread_attr_setinheritsched(&att, PTHREAD_EXPLICIT_SCHED); // set other sched
			//pthread_attr_setschedpolicy(&att, SCHED_DEADLINE); // specify which sched
			break;
		default:
			break;
	}
	sp.sched_priority = tp.priority; // set priority inside task struct
	pthread_attr_setschedparam(&att, &sp); // set priority
	
	err = pthread_create(&tid, &att, task, &tp); // create thread
	
	return err;
}

int task_init2(struct task_param *tp, struct sched_param *sp, pthread_attr_t att, pthread_t *tid, int sched, void* task)
// initialize and create a thread with: arg1=task_param, arg2=sched_param, arg3=pthread_attr, arg4=pthread_t, arg5=scheduler (1=FIFO, 2=RR, 3=DEADLINE, anything else=OTHER)
// better version than task_init
{
	int err;
	
	pthread_attr_init(&att); // initialize parameters 
		
	switch (sched)
	{
		case 1:
			pthread_attr_setinheritsched(&att, PTHREAD_EXPLICIT_SCHED); // set other sched
			pthread_attr_setschedpolicy(&att, SCHED_FIFO); // specify which sched
			break;
		case 2:
			pthread_attr_setinheritsched(&att, PTHREAD_EXPLICIT_SCHED); // set other sched
			pthread_attr_setschedpolicy(&att, SCHED_RR); // specify which sched
			break;
		case 3:
			pthread_attr_setinheritsched(&att, PTHREAD_EXPLICIT_SCHED); // set other sched
			//pthread_attr_setschedpolicy(&att, SCHED_DEADLINE); // specify which sched
			break;
		default:
			break;
	}
	sp->sched_priority = tp->priority; // set priority inside task struct
	pthread_attr_setschedparam(&att, sp); // set priority
	
	err = pthread_create(tid, &att, task, tp); // create thread
	
	return err;
}

void create_mutex(pthread_mutex_t mux, pthread_mutexattr_t mux_attr, int prio, int ceil)
// creates mutex with priority method in arg3: 1=PI, 2=HLP, else=none. arg4=ceiling
{
	switch (prio)
	{
	case 1:
		pthread_mutexattr_init(&mux_attr);
		pthread_mutexattr_setprotocol(&mux_attr, PTHREAD_PRIO_INHERIT);
		break;
		
	case 2:
		pthread_mutexattr_init(&mux_attr);
		pthread_mutexattr_setprotocol(&mux_attr, PTHREAD_PRIO_PROTECT);
		pthread_mutexattr_setprioceiling(&mux_attr, ceil);
		break;
		
	default:
		pthread_mutexattr_init(&mux_attr);
		pthread_mutexattr_setprotocol(&mux_attr, PTHREAD_PRIO_NONE);
		break;
	}

	pthread_mutex_init(&mux, &mux_attr);
}

int sign(float x)
// returns the sign of arg1. >0=1, <0=-1
{
	if (x < 0)
		return -1;
	else
		return 1;
}

float min(float a, float b)
// computes the min of 2 args
{
	if (a < b)
		return a;
	else
		return b;
}

float max(float a, float b)
// computes the max of 2 args
{
	if (a > b)
		return a;
	else
		return b;
}

float randf(float min, float max)
// generates a random number between min and max
{
	float	num, range;

	range = max - min + 1;		
	num = rand()/(1.0 + RAND_MAX)*range + min;

	return num;
}

int is_close(float a, float b, float tol)
// returns 1 if a = b +-tol
{
	if (b - tol < a && a < b + tol)
		return 1;
	else
		return 0;
}

 

