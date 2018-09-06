/*----------------------------------------------------------------------------*/
// SPACE LIBRARY --- SOURCE
// Useful functions for handling space objects
/*----------------------------------------------------------------------------*/

#define _GNU_SOURCE
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "space_lib.h"


/*----------------------------------------------------------------------------*/
/*Functions Definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void stars_generator(struct BITMAP **screen)
// Generates the screen background on the passed arg1
{
	int i = 0, j = 0;
	
	for (i = 0; i <= XWIN; i++)
	{
		for (j = 0; j <= YWIN; j++)
		{
			if (rand() > 0.995*RAND_MAX)
			{
				putpixel(*screen, i, j, 15);
			}
		}
	}
}
/*
float fix2rad(float x)
{
	x = x/256*2*M_PI;
	return x;
}
*/
float constrain_angle(float x, float max)
// constrain the angle in arg1 in [0,arg2]
{
    x = fmod(x,max);
    if (x < 0)
        x += max;
    return x;

}

float find_rotation_error(float ref, float rad)
// finds the correct rotation error between two angles
{
	float err;
	
	err = ref - rad;
	if (err < M_PI && err > -M_PI)
	{
		err = ref - rad;
	}
	else if (err > M_PI)
	{
		err -= 2*M_PI;
	}
	else if (err <= -M_PI)
	{
		err += 2*M_PI;
	}
	
	return err;
}

float find_apoapsis(float x, float y_vel)
// computes the apoapsis given arg1=current altitude and arg2=vertical velocity
{
	float apo;
	apo = x + y_vel*y_vel/(2*G_MOON);
	
	return apo;
}

float find_timetoapoapsis(float y_vel)
// find the time to apoapsis given arg1=vertical velocity
{
	float t;
	t = y_vel/G_MOON;
	
	return t;
}











