/*----------------------------------------------------------------------------*/
// SPACE LIBRARY --- HEADER
// Useful functions for handling space objects
/*----------------------------------------------------------------------------*/

#ifndef BALL_LIB_H
#define BALL_LIB_H

/*----------------------------------------------------------------------------*/


#include <time.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include "allegro_lib.h"


/*----------------------------------------------------------------------------*/
/*Constants-------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#define G_MOON 1.6279039 // gravity accel moon [m/s^2]
#define M_LEM 16279.039 // lem mass (supposed constant) [kg]
#define ASTBELT_UP 2000 // asteroid belt upper limit [m]
#define ASTBELT_LW 1800 // asteroid belt lower limit [m]
#define MAX_AST 12 // maximum number of asteroids
#define F1 45040 // main engine thrust [N]
#define F2 445 // rcs engine thrust [N]
#define CSM_ORBITAL_Y 93020*0.04 // orbital altitude of the csm [m]
#define CSM_ORBITAL_VEL 1610.83*0.03 // orbital velocity of the csm [m/s]


/*----------------------------------------------------------------------------*/
/*Structures------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

struct states1
{
	float	x, y, theta;
	float	x_vel, y_vel, theta_vel;
	float	main_eng; // 1=main engine on, 0=main engine off
	float	rot_rcs; // 1=clockwise, -1=counter clockwise
	float 	updwn_rcs; // 1=up, -1=down
	float	lr_rcs; // 1=right, -1=left
};

struct states2
{
	float	x, y, theta;
	float	x_vel, y_vel, theta_vel;
};

/*----------------------------------------------------------------------------*/
/*Functions Prototipes--------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void stars_generator(struct BITMAP **screen);
float constrain_angle(float x, float max);
float find_rotation_error(float ref, float angle);
float find_apoapsis(float x, float y_vel);
float find_timetoapoapsis(float y_vel);



/*----------------------------------------------------------------------------*/

#endif

