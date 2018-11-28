#include <allegro.h> //`
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <semaphore.h>
#include <stdbool.h>

#include "pthread_lib.h"
#include "allegro_lib.h"
#include "space_lib.h"

//#define _GNU_SOURCE
#define TSCALE 1


/*----------------------------------------------------------------------------*/
/*Gobal variables-------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
struct		states1 lem_states;
struct		states2 ast_states[MAX_AST];
struct		states2	csm_states;
int			end = 0; // flag for exit 0=no, 1=exit
float		apoapsis = CSM_ORBITAL_Y; // lem apoapsis
bool		autopilot = 0; // flag for autopilot, true=0=ON, false=1=OFF
int			not_landed = 1; // flag if landed, 1=no, 0=landed
//pthread_t	tid_ast[MAX_AST]; // thread ids for asteroids
int			n_ast = 0; // number of spawned asteroids
int			csm_spawned = 0; // flag if cms is spawned 0=no, 1=yes
int			vect_selector = 0; // 0=none, 1=surface, -1=docking
char		status1[100] = "--";
char		status2[100] = "--";
int			exploded = 0; // flag if lem is destroyed, 1=destroyed, 0=not
int			mode; // switch for quickly loading a starting point
	
	
/*----------------------------------------------------------------------------*/
/*Semaphores------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
pthread_mutex_t		mux_lem, mux_end, mux_autop, mux_land, mux_ast, mux_csm, mux_status;
pthread_mutexattr_t	mux_attr1;


/*----------------------------------------------------------------------------*/
/*Functions definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
void *display_task(void *param); 
void *input_task(void *param); 
void *lem_task(void *param);
void *ast_task(void *param); 
void *csm_task(void *param); 
void init_lem();
void init_csm();
void handle_landing(float *theta, float *x_vel, float *y_vel, float *theta_vel);
void handle_docking(float lem_theta, float lem_x, float lem_y, float rel_x_vel, float rel_y_vel, float csm_x);
void descent_autopilot1();
void ascent_autopilot1();
void lem_manual_controls();
void gui_controls();
void update_bottom_cp(float x, float y, float theta_deg, float x_vel, float y_vel, float theta_vel, float x_csm, int csm_spawned_loc);
void update_right_cp();
void update_minimap();
void create_ast_task();
int check_collision_path(float vref, int descent);


/*----------------------------------------------------------------------------*/
/*Main Function---------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
	// argument passing
	if (argc != 2)
	{
		printf("Correct usage: %s <mode>\n", argv[0]);
		printf("<0> = complete\n");
		printf("<1> = descent asteroid belt\n");
		printf("<2> = landing\n");
		printf("<3> = ascent asteroid belt\n");
		printf("<4> = orbital insertion\n");
		printf("<5> = docking\n");
		exit(-1);
	}
	mode = atoi(argv[1]);
	
	// initialize gui and semaphores
	init_gui();
	create_mutex(mux_lem, mux_attr1, 1, 70);	
	create_mutex(mux_end, mux_attr1, 1, 70);	
	create_mutex(mux_autop, mux_attr1, 1, 70);	
	create_mutex(mux_land, mux_attr1, 1, 70);	
	create_mutex(mux_ast, mux_attr1, 1, 70);	
	create_mutex(mux_csm, mux_attr1, 1, 70);	
	create_mutex(mux_status, mux_attr1, 1, 70);	
	pthread_mutexattr_destroy(&mux_attr1);
	
	int		err_lem, err_display, err_input, err_ast, err_csm;
	struct	task_param	tp_lem, tp_display, tp_input, tp_ast, tp_csm;
	struct	sched_param	sp_lem, sp_display, sp_input, sp_ast, sp_csm;
	pthread_t		tid_lem, tid_display, tid_input, tid_ast, tid_csm;
	pthread_attr_t	attr_lem, attr_display, attr_input, attr_ast, attr_csm;
	
	// start display task
	tp_display.arg = 1;
	tp_display.period = 30;
	tp_display.deadline = 100; 
	tp_display.priority = 70;
	tp_display.missCount = 0;
	err_display = task_init2(&tp_display, &sp_display, attr_display, &tid_display, 2, display_task);
	printf("\n-- display task created -- err=%d --\n\n", err_display);
	
	// start input task
	tp_input.arg = 1;
	tp_input.period = 30;
	tp_input.deadline = 100; 
	tp_input.priority = 50;
	tp_input.missCount = 0;
	err_input = task_init2(&tp_input, &sp_input, attr_input, &tid_input, 2, input_task);
	printf("\n-- input task created -- err=%d --\n\n", err_input);
	
	// start lem task
	tp_lem.arg = 1;
	tp_lem.period = 30;
	tp_lem.deadline = 60; 
	tp_lem.priority = 70;
	tp_lem.missCount = 0;
	err_lem = task_init2(&tp_lem, &sp_lem, attr_lem, &tid_lem, 2, lem_task);
	printf("\n-- lem task created -- err=%d --\n\n", err_lem);
	
	// start asteroids task
	tp_ast.arg = 1;
	tp_ast.period = 30;
	tp_ast.deadline = 60; 
	tp_ast.priority = 70;
	tp_ast.missCount = 0;
	err_ast = task_init2(&tp_ast, &sp_ast, attr_ast, &tid_ast, 2, ast_task);
	printf("\n-- asteroids task created -- err=%d --\n\n", err_ast);
	
	// start csm task
	tp_csm.arg = 1;
	tp_csm.period = 30;
	tp_csm.deadline = 60; 
	tp_csm.priority = 70;
	tp_csm.missCount = 0;
	err_csm = task_init2(&tp_csm, &sp_csm, attr_csm, &tid_csm, 2, csm_task);
	printf("\n-- csm task created -- err=%d --\n\n", err_csm);
		
	
	
	
	/*----------------------------------------------------------------------------*/
	/*Tasks Running---------------------------------------------------------------*/
	/*----------------------------------------------------------------------------*/
	
	
	
	
	// join all tasks
	pthread_join(tid_lem, NULL);
	pthread_join(tid_display, NULL);
	pthread_join(tid_input, NULL);
	pthread_join(tid_ast, NULL);
	pthread_join(tid_csm, NULL);
	
	// delete semaphores
	pthread_mutex_destroy(&mux_lem);
	pthread_mutex_destroy(&mux_end);
	pthread_mutex_destroy(&mux_autop);
	pthread_mutex_destroy(&mux_land);
	pthread_mutex_destroy(&mux_ast);
	pthread_mutex_destroy(&mux_csm);
	
	// exit allegro
	allegro_exit();
	
	return 0;
}


/*----------------------------------------------------------------------------*/
/*Function Definitions--------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void *display_task(void *param) 
// function for the dispaly task
{
	struct	task_param	*tp;
	struct	timespec t_now;

	clock_gettime(CLOCK_MONOTONIC, &t_now);
	tp = (struct task_param *)param;


	// DO STUFF ONLY ONCE
	float	x, x_vel, x_ast[MAX_AST], x_csm, x_vel_csm;
	float	y, y_vel, y_ast[MAX_AST], y_csm; 
	float	x_land = -1;
	float	main_eng, rot_rcs, updwn_rcs, lr_rcs;
	float	theta = 0, theta_vel, theta_deg = 0, theta_rad = 0, theta_ast[MAX_AST], theta_csm;
	float 	x_print_ast, y_print_ast, x_print_csm, y_print_csm, stretch, x_print_muz, y_print_muz;
	int 	i, n_ast_loc, csm_spawned_loc, color, vect_sel_loc, flag_expl = 0, exploded_loc;

	// load bitmaps
	BITMAP *lem, *ground_craters, *ast, *csm;
	BITMAP *expl, *maineng_muz, *rcs_muz;
	lem = load_bitmap("bitmaps/lem2.bmp", NULL);
	ground_craters = load_bitmap("bitmaps/ground_craters.bmp", NULL);
	ast = load_bitmap("bitmaps/ast1.bmp", NULL);
	csm = load_bitmap("bitmaps/csm2.bmp", NULL);
	expl = load_bitmap("bitmaps/exp1.bmp", NULL);
	maineng_muz = load_bitmap("bitmaps/eng_muz.bmp", NULL);
	rcs_muz = load_bitmap("bitmaps/rcs_muz.bmp", NULL);
	
	// initialize bg
	BITMAP *bg; 
	bg = create_bitmap(XWIN, YWIN);
	clear_bitmap(bg);
	stars_generator(&bg);
	blit(bg, screen, 0, 0, 0, 0, XWIN, YWIN);
	
	// initialize screen buffer
	BITMAP *screen_buff; 
	screen_buff = create_bitmap(XWIN, YWIN);
	blit(bg, screen, 0, 0, 0, 0, XWIN, YWIN);
	
	// setup right control panel with controls
	update_right_cp();


	set_period(tp);
	
	do
	{		
		clock_gettime(CLOCK_MONOTONIC, &t_now);
	
		// DO STUFF PERIODICALLY
		// read lem states
		pthread_mutex_lock(&mux_lem);
		x = lem_states.x;
		y = lem_states.y;
		theta = lem_states.theta;
		x_vel = lem_states.x_vel;
		y_vel = lem_states.y_vel;
		theta_vel = lem_states.theta_vel;
		main_eng = lem_states.main_eng;
		rot_rcs = lem_states.rot_rcs;
		updwn_rcs = lem_states.updwn_rcs;
		lr_rcs = lem_states.lr_rcs;
		pthread_mutex_unlock(&mux_lem);
		theta_deg = constrain_angle(theta, 2*M_PI)/M_PI*180.0;
		
		//read ast states
		pthread_mutex_lock(&mux_ast);
		n_ast_loc = n_ast;
		exploded_loc = exploded;
		if (n_ast_loc > 0)
		{
			for (i = 0; i < n_ast_loc; i++)
			{
				x_ast[i] = ast_states[i].x;
				y_ast[i] = ast_states[i].y;
				theta_ast[i] = ast_states[i].theta;
			}
		}
		pthread_mutex_unlock(&mux_ast);
		
		// read csm states
		pthread_mutex_lock(&mux_csm);
		x_csm = csm_states.x;
		y_csm = csm_states.y;
		theta_csm = csm_states.theta;
		x_vel_csm = csm_states.x_vel;
		csm_spawned_loc = csm_spawned;
		pthread_mutex_unlock(&mux_csm);
		
		// read vector selector
		pthread_mutex_lock(&mux_autop);
		vect_sel_loc = vect_selector; // 1=none, 2=surface, 3=docking
		pthread_mutex_unlock(&mux_autop);
		
		// create buffer
		blit(bg, screen_buff, 0, 0, 0, 0, bg->w, bg->h);
		
		// add lem or explosion
		if (exploded_loc)
		{
			stretch = min(80, flag_expl*10);
			stretch_sprite(screen_buff, expl, XWIN/2 - (0.5*stretch), YWIN/2 - (0.5*stretch), stretch, stretch);
			
			flag_expl++;
			if (flag_expl > 20)
			{
				pthread_mutex_lock(&mux_end);
				end = 1;
				pthread_mutex_unlock(&mux_end);
			}
		}
		else 
		{
			rotate_sprite(screen_buff, lem, XWIN/2 - lem->w/2, YWIN/2 - lem->h/2, ftofix(theta/(2*M_PI)*256));
		}
		
		if (main_eng && !exploded_loc) // main eng on
		{
			x_print_muz = XWIN/2 - 28*sin(theta) - maineng_muz->w/2;
			y_print_muz = YWIN/2 + 28*cos(theta) - maineng_muz->h/2;
			rotate_sprite(screen_buff, maineng_muz, x_print_muz, y_print_muz, ftofix(theta/(2*M_PI)*256));
		}
		
		if(rot_rcs > 0.01 && !exploded_loc) // cw
		{
			// right rcs
			x_print_muz = XWIN/2 + 14*sin(theta) + 22*cos(theta) - rcs_muz->w/2;
			y_print_muz = YWIN/2 - 14*cos(theta) + 22*sin(theta) - rcs_muz->h/2; 
			rotate_sprite(screen_buff, rcs_muz, x_print_muz, y_print_muz, ftofix((theta + M_PI)/(2*M_PI)*256));
			// left rcs
			x_print_muz = XWIN/2 - 20*cos(theta) - rcs_muz->w/2;
			y_print_muz = YWIN/2 - 20*sin(theta) - rcs_muz->h/2; 
			rotate_sprite(screen_buff, rcs_muz, x_print_muz, y_print_muz, ftofix((theta)/(2*M_PI)*256));
		}
		else if(rot_rcs < -0.01 && !exploded_loc) // ccw
		{
			// right rcs
			x_print_muz = XWIN/2 + 22*cos(theta) - rcs_muz->w/2;
			y_print_muz = YWIN/2 + 22*sin(theta) - rcs_muz->h/2; 
			rotate_sprite(screen_buff, rcs_muz, x_print_muz, y_print_muz, ftofix((theta)/(2*M_PI)*256));
			// left rcs
			x_print_muz = XWIN/2 + 14*sin(theta) - 20*cos(theta) - rcs_muz->w/2;
			y_print_muz = YWIN/2 - 14*cos(theta) - 20*sin(theta) - rcs_muz->h/2; 
			rotate_sprite(screen_buff, rcs_muz, x_print_muz, y_print_muz, ftofix((theta + M_PI)/(2*M_PI)*256));
		} 
		
		if(lr_rcs > 0.01 && !exploded_loc) // right
		{
			x_print_muz = XWIN/2 + 7*sin(theta) - 24*cos(theta) - rcs_muz->w/2;
			y_print_muz = YWIN/2 - 7*cos(theta) - 24*sin(theta) - rcs_muz->h/2; 
			rotate_sprite(screen_buff, rcs_muz, x_print_muz, y_print_muz, ftofix((theta + 0.5*M_PI)/(2*M_PI)*256));
		}
		else if(lr_rcs < -0.01 && !exploded_loc) // left
		{
			x_print_muz = XWIN/2 + 7*sin(theta) + 25*cos(theta) - rcs_muz->w/2;
			y_print_muz = YWIN/2 - 7*cos(theta) + 25*sin(theta) - rcs_muz->h/2; 
			rotate_sprite(screen_buff, rcs_muz, x_print_muz, y_print_muz, ftofix((theta + 1.5*M_PI)/(2*M_PI)*256));
		}
		
		if(updwn_rcs > 0.01 && !exploded_loc) // up 
		{
			// right rcs
			x_print_muz = XWIN/2 + 22*cos(theta) - rcs_muz->w/2;
			y_print_muz = YWIN/2 + 22*sin(theta) - rcs_muz->h/2; 
			rotate_sprite(screen_buff, rcs_muz, x_print_muz, y_print_muz, ftofix((theta)/(2*M_PI)*256));
			// left rcs
			x_print_muz = XWIN/2 - 20*cos(theta) - rcs_muz->w/2;
			y_print_muz = YWIN/2 - 20*sin(theta) - rcs_muz->h/2; 
			rotate_sprite(screen_buff, rcs_muz, x_print_muz, y_print_muz, ftofix((theta)/(2*M_PI)*256));
		} 
		else if(updwn_rcs < -0.01 && !exploded_loc) // down
		{
			// right rcs
			x_print_muz = XWIN/2 + 14*sin(theta) + 22*cos(theta) - rcs_muz->w/2;
			y_print_muz = YWIN/2 - 14*cos(theta) + 22*sin(theta) - rcs_muz->h/2; 
			rotate_sprite(screen_buff, rcs_muz, x_print_muz, y_print_muz, ftofix((theta + M_PI)/(2*M_PI)*256));
			// left rcs
			x_print_muz = XWIN/2 + 14*sin(theta) - 20*cos(theta) - rcs_muz->w/2;
			y_print_muz = YWIN/2 - 14*cos(theta) - 20*sin(theta) - rcs_muz->h/2; 
			rotate_sprite(screen_buff, rcs_muz, x_print_muz, y_print_muz, ftofix((theta + M_PI)/(2*M_PI)*256));
		}
		
		// add ground 
		if (y < 0.1*YWIN/2)
		{
			if (x_land < 0)
			{	
				x_land = x;
			}
			blit(ground_craters, screen_buff, (ground_craters->w)/2 + 10*(x - x_land), 0, 0, YWIN/2 + 10*y, XWIN, YWIN/2);
		}
		
		// add asteroids
		if (n_ast_loc > 0)
		{
			for (i = 0; i < n_ast_loc; i++)
			{
				x_print_ast = (XWIN/2 + 10*(fabs(x_ast[i]) - fabs(x))) - ast->w/2; //remove fabs?
				y_print_ast = YWIN/2 + 10*(fabs(y) - fabs(y_ast[i])) - ast->h/2;
				rotate_sprite(screen_buff, ast, x_print_ast, y_print_ast, ftofix(theta_ast[i]/(2*M_PI)*256));
					
			}
		}
		
		// add csm
		x_print_csm = XWIN/2 + 10*(fabs(x_csm) - fabs(x)) - csm->w/2; 
		y_print_csm = YWIN/2 - 10*(fabs(y_csm) - fabs(y)) - csm->h/2;
		if (fabs(x_csm - x)*10 < XWIN/2 + 50 && fabs(y_csm - y)*10 < YWIN/2 + 50 && csm_spawned_loc)
		{
			rotate_sprite(screen_buff, csm, x_print_csm, y_print_csm, ftofix(theta_csm/(2*M_PI)*256));
		}
		
		// add surface velocity vector // 1=none, 2=surface, 3=docking
		if (vect_sel_loc == 2) 
		{	
			line(screen_buff, XWIN/2, YWIN/2, XWIN/2 + 5*(x_vel), YWIN/2 + 5*(-y_vel), 4);
		}		
		// add relative to csm velocity vector and docking ports alignment vector
		else if (vect_sel_loc == 3 && csm_spawned_loc)
		{
			// alignment vector
			line(screen_buff, XWIN/2 + sin(theta)*24, YWIN/2 - cos(theta)*24, x_print_csm - 36 + csm->w/2, y_print_csm + csm->h/2, 9);
			
			// relative vel vector
			color = makecol(255, 133, 51);
			line(screen_buff, XWIN/2, YWIN/2, XWIN/2 + 50*(x_vel - x_vel_csm), YWIN/2 + 50*(-y_vel), color);
			
		}

		// update screen
		blit(screen_buff, screen, 0, 0, 0, 0, screen_buff->w, screen_buff->h);
		
		// update bottom control panel
		update_bottom_cp(x, y, theta_deg, x_vel, y_vel, theta_vel, x_csm, csm_spawned_loc);
		
		// update altitude map
		update_minimap();
		
		
		
		if (deadline_miss(tp))
		{
			printf("\n\n --- display task missed a deadline! --- \n\n");
		}
		wait_for_period(tp);
	
	} while (!end);

	pthread_exit(0);
}

void *input_task(void *param)  
// function for the input task
{	
	struct task_param	*tp;
	struct timespec t_now;
	
	clock_gettime(CLOCK_MONOTONIC, &t_now);
	tp = (struct task_param *)param;


	// DO STUFF ONLY ONCE
	float y, y_vel;
	bool autop;
	int not_landed_loc;
	
	
	set_period(tp);
	
	do
	{		
		clock_gettime(CLOCK_MONOTONIC, &t_now);
	
		// DO STUFF PERIODICALLY
		// reset engine and rcs flags
		pthread_mutex_lock(&mux_lem);
		y = lem_states.y;
		y_vel = lem_states.y_vel;
		lem_states.main_eng = 0;
		lem_states.rot_rcs = 0;
		lem_states.updwn_rcs = 0;
		lem_states.lr_rcs = 0;
		not_landed_loc = not_landed;
		pthread_mutex_unlock(&mux_lem);
		
		// read flags
		pthread_mutex_lock(&mux_autop);
		autop = autopilot;
		pthread_mutex_unlock(&mux_autop);
		
		gui_controls();
		
		// Autopilot control
		if (autop)
		{
			if (not_landed_loc)
			{
				descent_autopilot1(); 
			}
			else
			{
				ascent_autopilot1(); 
			}
		}
		
		// Manual control
		else
		{
			// read key and interpret command
			lem_manual_controls();
			
			// update status message
			pthread_mutex_lock(&mux_status);
			sprintf(status1, "Manual Control");
			pthread_mutex_unlock(&mux_status);
			
			// update status2 after launch
			if (y > 6 && y < CSM_ORBITAL_Y - 100)
			{
				pthread_mutex_lock(&mux_status);
				sprintf(status2, " -- ");
				pthread_mutex_unlock(&mux_status);
			}
			
			// update status2 inside asteroid belt
			if (y > ASTBELT_LW - 100 && y < ASTBELT_UP + 100 && ((not_landed_loc && check_collision_path(y_vel, 1) != 0) || (!not_landed_loc && check_collision_path(y_vel, 0) != 0)))
			{
				pthread_mutex_lock(&mux_status);
				sprintf(status2, " COLLISION ALERT! COLLISION ALERT! ");
				pthread_mutex_unlock(&mux_status);
			}
		}
	
	
		if (deadline_miss(tp))
		{
			printf("\n\n --- input task missed a deadline! --- \n\n");
		}
		wait_for_period(tp);
	
	} while (!end);
	
	pthread_exit(0);
	
}

void *lem_task(void *param)
// function for the lem task
{
	struct	task_param	*tp;
	struct	timespec t_now;	

	clock_gettime(CLOCK_MONOTONIC, &t_now);
	tp = (struct task_param *)param;


	// DO STUFF ONLY ONCE
	float	x, y, theta, theta_rad, y_apo;
	float	x_vel, y_vel, theta_vel;
	float	accel_eng, accel_rcs, theta_accel, mod = 1;
	float	main_eng, rot_rcs, updwn_rcs, lr_rcs, theta_acc, x_accel, y_accel; 
	float	dt;
	int 	landed_loc, n_ast_loc, end_ast_loc;
	
	// create constants for linear and angular accel
	init_lem(); // set initial values for pos, vel etc
	dt = TSCALE*0.001*(float)tp->period;
	accel_eng = 2*F1/M_LEM; 
	accel_rcs = 8*F2/M_LEM;
	theta_accel = 2*F2*2.13/1000;
	
	// create data log file
	FILE *file = fopen("data_log.txt", "w");
	if (file == NULL)
	{
		printf("Error opening the data log file!\n");
	}
	fprintf(file, "FORMAT: y[m], vy[m/s], vx[m/s] --- DT[s]=%f\n", dt);
	
	
	set_period(tp);
	
	do
	{		
		clock_gettime(CLOCK_MONOTONIC, &t_now);
	
		// DO STUFF PERIODICALLY
		// read lem global vars and copy in LOCAL vars
		pthread_mutex_lock(&mux_lem);
		theta = lem_states.theta;
		x = lem_states.x;
		y = lem_states.y;
		x_vel = lem_states.x_vel;
		y_vel= lem_states.y_vel;
		theta_vel = lem_states.theta_vel;
		main_eng = lem_states.main_eng; 
		rot_rcs = lem_states.rot_rcs; 
	 	updwn_rcs = lem_states.updwn_rcs; 
		lr_rcs = lem_states.lr_rcs; 
		pthread_mutex_unlock(&mux_lem);
		pthread_mutex_lock(&mux_land);
		landed_loc = not_landed;
		pthread_mutex_unlock(&mux_land);
		
		// read lem global vars and copy in LOCAL vars		
		pthread_mutex_lock(&mux_ast);
		n_ast_loc = n_ast;
		pthread_mutex_unlock(&mux_ast);
		
		theta = constrain_angle(theta, 2.0*M_PI);
		
		// when the lem is descending
		if (not_landed)
		{			
			// compute apoapsis
			y_apo = CSM_ORBITAL_Y;
			// compute gravity modifier
			mod = 1;
		}
		
		// after the lem has landed and is ascending to dock with csm
		else
		{
			// compute apoapsis
			y_apo = find_apoapsis(y, y_vel);
			// compute gravity modifier
			mod = pow((CSM_ORBITAL_VEL - fabs(x_vel))/(CSM_ORBITAL_VEL), 4);
			
		}
		
		// calculate new states in LOCAL vars
		theta += rot_rcs*0.5*theta_accel*dt*dt + theta_vel*dt;
		theta_vel += rot_rcs*theta_accel*dt;
		x += 0.5*(sin(theta)*(updwn_rcs*accel_rcs + main_eng*accel_eng) + cos(theta)*accel_rcs*lr_rcs)*dt*dt + x_vel*dt;
		y += 0.5*(cos(theta)*(updwn_rcs*accel_rcs + main_eng*accel_eng) - sin(theta)*accel_rcs*lr_rcs)*dt*dt + y_vel*dt;
		x_vel += (sin(theta)*(updwn_rcs*accel_rcs + main_eng*accel_eng) + cos(theta)*accel_rcs*lr_rcs)*dt;
		y_vel += (cos(theta)*(updwn_rcs*accel_rcs + main_eng*accel_eng) - sin(theta)*accel_rcs*lr_rcs - G_MOON*mod)*dt;
		
		// handle ground landing
		if (y <= 2.5) // account for bitmap dimension
		{		
			handle_landing(&theta, &x_vel, &y_vel, &theta_vel);
		}
	
		// update states in GLOBAL vars
		pthread_mutex_lock(&mux_lem);
		lem_states.theta = theta;
		lem_states.theta_vel = theta_vel;
		lem_states.x = x;
		lem_states.y = y;
		lem_states.x_vel = x_vel;
		lem_states.y_vel= y_vel;
		apoapsis = y_apo;
		pthread_mutex_unlock(&mux_lem);
		
		// log data to file
		fprintf(file, "y=%f vy=%f vx=%f\n", y, y_vel, x_vel);
		
		if (deadline_miss(tp))
		{
			printf("\n\n --- lem task missed a deadline! --- \n\n");
		}
		wait_for_period(tp);
	
	} while (!end);
	
	pthread_exit(0);
}

void *ast_task(void *param)
//functionfor the astroid task
{
	struct task_param	*tp;
	struct timespec 	t_now;
	
	clock_gettime(CLOCK_MONOTONIC, &t_now);
	tp = (struct task_param *)param;


	// DO STUFF ONLY ONCE
	int		n_ast_loc, not_landed_loc;
	float	x[MAX_AST], y[MAX_AST], theta[MAX_AST], x_vel[MAX_AST], theta_vel[MAX_AST], dt;
	float	lem_x, lem_y;
	int 	i;
	int		asteroids_y_asc[9] = {ASTBELT_UP - 200, ASTBELT_UP - 170, ASTBELT_UP - 140, ASTBELT_UP - 120, ASTBELT_UP - 100, ASTBELT_UP - 80, ASTBELT_UP - 60, ASTBELT_UP - 30, ASTBELT_UP};
	int		asteroids_y_desc[9] = {ASTBELT_UP, ASTBELT_UP - 30, ASTBELT_UP - 60, ASTBELT_UP - 80, ASTBELT_UP - 100, ASTBELT_UP - 120, ASTBELT_UP - 140, ASTBELT_UP - 170, ASTBELT_UP - 200};
	
	dt = TSCALE*0.001*(float)tp->period;
	
	
	set_period(tp);
	
	do
	{		
		clock_gettime(CLOCK_MONOTONIC, &t_now);
	
		// DO STUFF PERIODICALLY
		// read number of active asteroids
		pthread_mutex_lock(&mux_ast);
		n_ast_loc = n_ast;
		pthread_mutex_unlock(&mux_ast);
	
		// read lem data
		pthread_mutex_lock(&mux_lem); 
		lem_x = lem_states.x;
		lem_y = lem_states.y;
		pthread_mutex_unlock(&mux_lem);
		pthread_mutex_lock(&mux_land); 
		not_landed_loc = not_landed;
		pthread_mutex_unlock(&mux_land);	

		// if inside asteroid belt and descending
		if (lem_y < ASTBELT_UP + 50 && lem_y > ASTBELT_LW - 100 && not_landed_loc)
		{	
			// spawns asteroids
			for (i = 0; i < MAX_AST; i++)
			{
				if (is_close(lem_y, asteroids_y_desc[i], 5) && n_ast_loc == i)
				{
					n_ast_loc = add_ast(n_ast_loc);
				}
			}
		}
	
		// if inside asteroid belt and ascending
		else if (lem_y < ASTBELT_UP + 50 && lem_y > ASTBELT_LW - 100 && !not_landed_loc)
		{
			// spawns asteroids
			for (i = 0; i < MAX_AST; i++)
			{
				if (is_close(lem_y, asteroids_y_asc[i], 5) && n_ast_loc == i)
				{
					n_ast_loc = add_ast(n_ast_loc);
				}
			}
		}	
		
		// read active asteroids data 
		pthread_mutex_lock(&mux_ast);
		for (i = 0; i < n_ast_loc; i++)
		{
			x[i] = ast_states[i].x;
			y[i] = ast_states[i].y;
			theta[i] = ast_states[i].theta;
			x_vel[i] = ast_states[i].x_vel;
			theta_vel[i] = ast_states[i].theta_vel;
		}
		pthread_mutex_unlock(&mux_ast);
	
		// compute all new x and theta
		for (i = 0; i < n_ast_loc; i++)
		{
			x[i] += x_vel[i]*dt;
			theta[i] += theta_vel[i]*dt;
		}
	
		// write new x and theta into gloabal vector
		pthread_mutex_lock(&mux_ast);
		for (i = 0; i < n_ast_loc; i++)
		{
			ast_states[i].theta = theta[i];
			ast_states[i].x = x[i];
		}
		pthread_mutex_unlock(&mux_ast);
	
		// collision detection
		for (i = 0; i < n_ast_loc; i++)
		{
			if ( fabs(lem_x - x[i]) < (32 + 16)/10 && fabs(lem_y - y[i]) < (24 + 16)/10)
			{
				if (!exploded)
				{
					printf("\nASTEROID COLLISION!! \n"); 
					pthread_mutex_lock(&mux_end);
					exploded = 1;
					pthread_mutex_unlock(&mux_end);
				}
			}
		}
		
		// update n_ast (global and local) when lem is out of belt
		if ((lem_y < ASTBELT_LW - 100 || lem_y > ASTBELT_UP + 100) && n_ast_loc != 0)
		{
			pthread_mutex_lock(&mux_ast);
			n_ast = 0;
			pthread_mutex_unlock(&mux_ast);
			n_ast_loc = 0;
		}	
						
		
		
	
		if (deadline_miss(tp))
		{
			printf("\n\n --- asteroids task missed a deadline! --- \n\n");
		}
		wait_for_period(tp);
	
	} while (!end);
	
	pthread_exit(0);
}

void *csm_task(void *param)
// function for the csm task
{
	struct task_param	*tp;
	struct timespec t_now;
	
	clock_gettime(CLOCK_MONOTONIC, &t_now);
	tp = (struct task_param *)param;


	// DO STUFF ONLY ONCE
	// define local states for csm and lem
	float lem_x, lem_y, lem_theta, lem_x_vel, lem_y_vel, lem_theta_vel;
	float csm_x, csm_y, csm_x_vel, dt;
	int not_landed_loc, csm_spawned_loc;
	
	dt = TSCALE*0.001*(float)tp->period;
	
	
	set_period(tp);
	
	do
	{		
		clock_gettime(CLOCK_MONOTONIC, &t_now);
	
		// DO STUFF PERIODICALLY
		// read lem data
		pthread_mutex_lock(&mux_lem); 
		lem_x = lem_states.x;
		lem_y = lem_states.y;
		lem_theta = lem_states.theta;
		lem_x_vel = lem_states.x_vel;
		lem_y_vel = lem_states.y_vel;
		lem_theta_vel = lem_states.theta_vel;
		pthread_mutex_unlock(&mux_lem);
		pthread_mutex_lock(&mux_land); 
		not_landed_loc = not_landed;
		pthread_mutex_unlock(&mux_land);	
		
		// if close to the csm orbit and ascending
		if (fabs(lem_y - CSM_ORBITAL_Y) < 200 && !not_landed_loc)
		{			
			pthread_mutex_lock(&mux_csm);
			csm_spawned_loc = csm_spawned;
			pthread_mutex_unlock(&mux_csm);
			
			if (fabs(lem_y - CSM_ORBITAL_Y) < 2.0 && !csm_spawned_loc && fabs(lem_x_vel - CSM_ORBITAL_VEL) < 1.0 && fabs(lem_y_vel) < 1.0)
			{
				// spawn csm and initialize states
				init_csm(); 
				csm_spawned_loc = 1;
			}
			
			// read csm data
			pthread_mutex_lock(&mux_csm); 
			csm_x = csm_states.x;
			csm_y = csm_states.y;
			csm_x_vel = csm_states.x_vel;
			csm_spawned_loc = csm_spawned;
			pthread_mutex_unlock(&mux_csm);
			
			// compute new csm x
			csm_x += csm_x_vel*dt;

			// update csm x
			pthread_mutex_lock(&mux_csm); 
			csm_states.x = csm_x;
			pthread_mutex_unlock(&mux_csm);
			
			// lem-csm docking detection
			if (fabs(csm_x - lem_x) < 6.1 && fabs(CSM_ORBITAL_Y - lem_y) < 4)
			{
				handle_docking(lem_theta, lem_x, lem_y, csm_x_vel - lem_x_vel, -lem_y_vel, csm_x);
			}
		}
	

		if (deadline_miss(tp))
		{
			printf("\n\n --- csm task missed a deadline! --- \n\n");
		}
		wait_for_period(tp);
	
	} while (!end);
	
	pthread_exit(0);
}

void init_lem()
// set initial values for LEM states
{	
	switch (mode)
	{
	case 0: // complete
		pthread_mutex_lock(&mux_lem);
		lem_states.x = 100;
		lem_states.y = CSM_ORBITAL_Y; 
		lem_states.theta = 1.85*M_PI; 
		lem_states.x_vel = CSM_ORBITAL_VEL; 
		lem_states.y_vel = -2; 
		lem_states.theta_vel = 0;
		lem_states.main_eng = 0;
		lem_states.rot_rcs = 0;
		lem_states.updwn_rcs = 0;
		lem_states.lr_rcs = 0;
		pthread_mutex_unlock(&mux_lem);	
		break;
	
	case 1: // before descent asteroid belt
		pthread_mutex_lock(&mux_lem);
		lem_states.x = 100;
		lem_states.y = 2050; 
		lem_states.theta = 0; 
		lem_states.x_vel = 0; 
		lem_states.y_vel = -2; 
		lem_states.theta_vel = 0;
		lem_states.main_eng = 0;
		lem_states.rot_rcs = 0;
		lem_states.updwn_rcs = 0;
		lem_states.lr_rcs = 0;
		pthread_mutex_unlock(&mux_lem);	
		break;
		
	case 2: // before landing
		pthread_mutex_lock(&mux_lem);
		lem_states.x = 1000;
		lem_states.y = 50;
		lem_states.theta = 0;
		lem_states.x_vel = 2; 
		lem_states.y_vel = 0; 
		lem_states.theta_vel = 0;
		lem_states.main_eng = 0;
		lem_states.rot_rcs = 0;
		lem_states.updwn_rcs = 0;
		lem_states.lr_rcs = 0;
		pthread_mutex_unlock(&mux_lem);	
		break;
	
	case 3: // before ascent asteroid belt
		pthread_mutex_lock(&mux_lem);
		lem_states.x = 1000;
		lem_states.y = 1750; 
		lem_states.theta = 0; 
		lem_states.x_vel = 0; 
		lem_states.y_vel = 10; 
		lem_states.theta_vel = 0;
		lem_states.main_eng = 0;
		lem_states.rot_rcs = 0;
		lem_states.updwn_rcs = 0;
		lem_states.lr_rcs = 0;
		pthread_mutex_unlock(&mux_lem);	
		pthread_mutex_lock(&mux_land); 
		not_landed = 0;
		pthread_mutex_unlock(&mux_land);
		break;
		
	case 4: // before orbital insertion 
		pthread_mutex_lock(&mux_lem);
		lem_states.x = 1000;
		lem_states.y = 2090; 
		lem_states.theta = 0; 
		lem_states.x_vel = 0; 
		lem_states.y_vel = 5; 
		lem_states.theta_vel = 0;
		lem_states.main_eng = 0;
		lem_states.rot_rcs = 0;
		lem_states.updwn_rcs = 0;
		lem_states.lr_rcs = 0;
		pthread_mutex_unlock(&mux_lem);	
		pthread_mutex_lock(&mux_land); 
		not_landed = 0;
		pthread_mutex_unlock(&mux_land);
		break;
	
	case 5: // ready to dock
		pthread_mutex_lock(&mux_lem);
		lem_states.x = 1000;
		lem_states.y = CSM_ORBITAL_Y; 
		lem_states.theta = M_PI/2; 
		lem_states.x_vel = CSM_ORBITAL_VEL; 
		lem_states.y_vel = 0; 
		lem_states.theta_vel = 0;
		lem_states.main_eng = 0;
		lem_states.rot_rcs = 0;
		lem_states.updwn_rcs = 0;
		lem_states.lr_rcs = 0;
		pthread_mutex_unlock(&mux_lem);	
		pthread_mutex_lock(&mux_land); 
		not_landed = 0;
		pthread_mutex_unlock(&mux_land);
		break;
	
	default: // complete
		pthread_mutex_lock(&mux_lem);
		lem_states.x = 100;
		lem_states.y = CSM_ORBITAL_Y; 
		lem_states.theta = 1.85*M_PI; 
		lem_states.x_vel = CSM_ORBITAL_VEL; 
		lem_states.y_vel = -2; 
		lem_states.theta_vel = 0;
		lem_states.main_eng = 0;
		lem_states.rot_rcs = 0;
		lem_states.updwn_rcs = 0;
		lem_states.lr_rcs = 0;
		pthread_mutex_unlock(&mux_lem);	
		break;
	}
}

void init_csm()
// set initial values for CSM states
{
	float lem_x;
	
	// lem data
	pthread_mutex_lock(&mux_lem); 
	lem_x = lem_states.x;
	pthread_mutex_unlock(&mux_lem);
	
	// set csm data
	pthread_mutex_lock(&mux_csm); 
	csm_states.x = lem_x + XWIN/20.0 + 20;
	csm_states.y = CSM_ORBITAL_Y;
	csm_states.x_vel = CSM_ORBITAL_VEL;
	csm_states.theta = M_PI*3/2;
	csm_states.theta_vel = 0;
	csm_spawned = 1;
	pthread_mutex_unlock(&mux_csm);
}

void handle_landing(float *theta, float *x_vel, float *y_vel, float *theta_vel)
// handles landing and explosion
{	
	if ((*theta > 0.34 && *theta < 5.93) || *y_vel < -5 || abs(*x_vel) > 2)
	{
		*x_vel = 0;
		*y_vel = 0;
		*theta_vel = 0;
		
		if (!exploded)
		{
			printf("\nLANDING FAILED!! \n"); 

			pthread_mutex_lock(&mux_end);
			exploded = 1;
			pthread_mutex_unlock(&mux_end);
		}
	}
	else
	{
		
		*x_vel = 0; 
		*y_vel = 0; 
		*theta_vel = .5*(*theta_vel);
			
		// adjust theta 
		if (*theta < 0.4)
		{
			*theta = .5*(*theta);
		}
		else
		{
			*theta += .5*(2*M_PI - *theta);
		}		
		
		// update flags
		pthread_mutex_lock(&mux_land); 
		not_landed = 0;
		pthread_mutex_unlock(&mux_land);
		pthread_mutex_lock(&mux_lem);
		apoapsis = 0;
		pthread_mutex_unlock(&mux_lem);	
		
		// update status message
		pthread_mutex_lock(&mux_status);
		sprintf(status2, "Landing successful - autopilot disengaged");
		pthread_mutex_unlock(&mux_status);
		
	}
}

void handle_docking(float lem_theta, float lem_x, float lem_y, float rel_x_vel, float rel_y_vel, float csm_x)
// handles landing and explosion
{
	if (fabs(lem_theta - M_PI/2) > 0.2 || fabs(rel_x_vel) > 0.3 || fabs(rel_y_vel) > 0.2 || fabs(csm_x - lem_x) < 6 || fabs(CSM_ORBITAL_Y - lem_y) > 0.5)
	{
		if (!exploded)
		{
			printf("\nDOCKING FAILED!! \n"); 
			pthread_mutex_lock(&mux_end);
			exploded = 1;
			pthread_mutex_unlock(&mux_end);	
		}
	}
	else
	{		
		// update lem states
		pthread_mutex_lock(&mux_lem);
		lem_states.y = CSM_ORBITAL_Y;
		lem_states.theta = M_PI/2;
		lem_states.x_vel = CSM_ORBITAL_VEL;
		lem_states.y_vel = 0;
		lem_states.theta_vel = 0;
		apoapsis = CSM_ORBITAL_Y;
		pthread_mutex_unlock(&mux_lem);	
		
		// disable autopilot
		pthread_mutex_lock(&mux_autop);
		autopilot = 0;
		pthread_mutex_unlock(&mux_autop);
			
		// update status message
		pthread_mutex_lock(&mux_status);
		sprintf(status2, "Docking successful - autopilot disengaged");
		pthread_mutex_unlock(&mux_status);
	}	
}

void descent_autopilot1()
// gravity turn controller
{
	float	Kp_theta = .07, Kp_y_vel = -0.1,  Kp_x_vel = 0.1, x_vel_off = -20;
	float	err_theta = 0, theta_ref;
	float	y, x_vel, y_vel, y_vel_ref, x_vel_ref, theta, theta_vel;
	int		check;
	
	pthread_mutex_lock(&mux_lem); 
	theta = lem_states.theta;
	y = lem_states.y;
	x_vel = lem_states.x_vel;
	y_vel = lem_states.y_vel;
	theta_vel = lem_states.theta_vel;
	pthread_mutex_unlock(&mux_lem);
	
	theta = constrain_angle(theta, 2*M_PI);
	
	// if outside asteroid belt align to velocity vector and burn
	if (y > ASTBELT_UP + 400 || y < ASTBELT_LW - 100)
	{
		// update status message
		pthread_mutex_lock(&mux_status);
		sprintf(status1, "Performing Gravity Turn descent");
		pthread_mutex_unlock(&mux_status);
		
		y_vel_ref = min(-3, -min(40, -y*Kp_y_vel));
		if (y > 200)
			x_vel_ref = min(1.0, y*Kp_x_vel + x_vel_off);
		else
			x_vel_ref = 0;
	
		theta_ref = M_PI*3/2 - atan2(y_vel,x_vel);
		theta_ref = constrain_angle(theta_ref, 2*M_PI);
		err_theta = find_rotation_error(theta_ref, theta);
	
		// regulate theta
		if (err_theta > 0.02 && theta_vel < err_theta*Kp_theta)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.rot_rcs = 1;
			pthread_mutex_unlock(&mux_lem);
		}
		else if (err_theta < -0.02 && theta_vel > err_theta*Kp_theta)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.rot_rcs = -1;
			pthread_mutex_unlock(&mux_lem);
		}		
	
		// when aligned to velocity vector then kill velocity w/ main engine	
		if (fabs(err_theta) < 0.2 && (y_vel < y_vel_ref || fabs(x_vel) > fabs(x_vel_ref) + 1 ))
		{
			// update status message
			if (y > 40)
			{
				pthread_mutex_lock(&mux_status);
				sprintf(status2, "Controlling vertical velocity");
				pthread_mutex_unlock(&mux_status);
			}
			else
			{
				pthread_mutex_lock(&mux_status);
				sprintf(status2, "Landing");
				pthread_mutex_unlock(&mux_status);
			}
			
			// main engine on
			pthread_mutex_lock(&mux_lem); 
			lem_states.main_eng = 1;
			pthread_mutex_unlock(&mux_lem);
		}
		// update status message
		else if (fabs(err_theta) > 0.3)
		{
			// update status message
			pthread_mutex_lock(&mux_status);
			sprintf(status2, "Aligning to velocity vector");
			pthread_mutex_unlock(&mux_status);
		}
		
		// small adjustments to horizontal velocity with rcs
		if ((theta > 6.2 || theta < 0.1) && fabs(x_vel) > 0.01)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.lr_rcs = -sign(x_vel);
			pthread_mutex_unlock(&mux_lem);
		}
		
		// disable autopilot when landed
		if (y < 3)
		{
			pthread_mutex_lock(&mux_autop); 
			autopilot = 0;
			pthread_mutex_unlock(&mux_autop);
		}
	}
	
	// if inside the asteroid belt regulate velocity and avoid asteroids
	else
	{
		// check if on a collision path
		check = check_collision_path(-5, 1);
		
		// update status message
		pthread_mutex_lock(&mux_status);
		sprintf(status1, "Navigating through asteroid belt");
		pthread_mutex_unlock(&mux_status);
	
		if (check == -1)
		{
			y_vel_ref = -1;
			// update status message
			pthread_mutex_lock(&mux_status);
			sprintf(status2, "Avoiding asteroid");
			pthread_mutex_unlock(&mux_status);
		}
		else if (check == 1)
		{
			y_vel_ref = -8;
			// update status message
			pthread_mutex_lock(&mux_status);
			sprintf(status2, "Avoiding asteroid");
			pthread_mutex_unlock(&mux_status);
		}
		else
		{
			y_vel_ref = min( -5, max(-30.0, -45.0/290*y + 89000.0/290)); 
			pthread_mutex_lock(&mux_status);
			sprintf(status2, "Controlling vertical velocity");
			pthread_mutex_unlock(&mux_status);
		}
			
		theta_ref = 0; 
		theta_ref = constrain_angle(theta_ref, 2*M_PI);
		err_theta = find_rotation_error(theta_ref, theta);

		// regulate theta
		if (err_theta > 0.02 && theta_vel < err_theta*Kp_theta)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.rot_rcs = 1;
			pthread_mutex_unlock(&mux_lem);
		}
		else if (err_theta < -0.02 && theta_vel > err_theta*Kp_theta)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.rot_rcs = -1;
			pthread_mutex_unlock(&mux_lem);
		}		
	
		// when aligned to velocity vector then kill velocity w/ main engine	
		if (fabs(err_theta) < 0.5 && y_vel < y_vel_ref)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.main_eng = 1;
			pthread_mutex_unlock(&mux_lem);
		}
		
		// if there is asteroid
		if (check != 0) 
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.lr_rcs = check;
			pthread_mutex_unlock(&mux_lem);
		}
		// small adjustments of horiz velocity with rcs
		else 
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.lr_rcs = -sign(x_vel);
			pthread_mutex_unlock(&mux_lem);
		}
	}
}

void ascent_autopilot1()
// guide the lem through the asterodis and then to the correct orbit to dock with the csm
{
	float	Kp_theta = .07;
	float	err_theta = 0, x_vel_err, y_vel_err, x_err, y_err;
	float	theta_ref, y_vel_ref, x_vel_ref, y_ref;
	float	x, y, x_vel, y_vel, theta, theta_vel, x_rel_vel;
	float	t_apo, y_apo;
	float	csm_x;
	int		check, csm_spawned_loc;
	
	// read lem data
	pthread_mutex_lock(&mux_lem); 
	theta = lem_states.theta;
	x = lem_states.x;
	y = lem_states.y;
	x_vel = lem_states.x_vel;
	y_vel = lem_states.y_vel;
	theta_vel = lem_states.theta_vel;
	pthread_mutex_unlock(&mux_lem);
	
	theta = constrain_angle(theta, 2*M_PI);
	
	// read csm data
	pthread_mutex_lock(&mux_csm);
	csm_spawned_loc = csm_spawned;
	csm_x = csm_states.x;
	pthread_mutex_unlock(&mux_csm);
	
	// compute apoapsis and time to apoapsis
	t_apo = find_timetoapoapsis(y_vel); //y_vel/G_MOON; 
	y_apo = find_apoapsis(y, y_vel); //y + y_vel*y_vel/(2*G_MOON); 
	
	// update global apoapsis
	pthread_mutex_lock(&mux_lem);
	apoapsis = y_apo;
	pthread_mutex_unlock(&mux_lem);
	
	// thrust upwards to build vertical positive vel (+50 max and +5 at the ast belt)
	if (y < ASTBELT_LW - 50)
	{
		// update status message
		pthread_mutex_lock(&mux_status);
		sprintf(status1, "Taking off");
		pthread_mutex_unlock(&mux_status);
		
		// set references
		y_vel_ref = max(5, -45.0/1750*y + 50);
		theta_ref = 0;
		err_theta = find_rotation_error(theta_ref, theta);
		
		// regulate theta
		if (err_theta > 0.02 && theta_vel < err_theta*Kp_theta)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.rot_rcs = 1;
			pthread_mutex_unlock(&mux_lem);
		}
		else if (err_theta < -0.02 && theta_vel > err_theta*Kp_theta)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.rot_rcs = -1;
			pthread_mutex_unlock(&mux_lem);
		}
		
		// regulate y_vel	
		if (fabs(err_theta) < 0.2 && y_vel < y_vel_ref)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.main_eng = 1;
			pthread_mutex_unlock(&mux_lem);
			pthread_mutex_lock(&mux_status);
			sprintf(status2, "Controlling vertical velocity");
			pthread_mutex_unlock(&mux_status);
		}
		
		// small adjustments to x_vel with rcs
		if ((theta > 6.2 || theta < 0.1) && fabs(x_vel) > 0.01)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.lr_rcs = -sign(x_vel);
			pthread_mutex_unlock(&mux_lem);
		}
	}
	
	// inside asteroid belt keep constant y_vel (+5) and avoid collisions with rcs
	else if (y > ASTBELT_LW - 50 && y < ASTBELT_UP + 100)
	{
		// check if on a collision path
		check = check_collision_path(+5, 0);
		
		// update status message
		pthread_mutex_lock(&mux_status);
		sprintf(status1, "Navigating through asteroid belt");
		pthread_mutex_unlock(&mux_status);
		
		// set y_vel reference
		if (check == -1)
		{
			y_vel_ref = 2;
			// update status message
			pthread_mutex_lock(&mux_status);
			sprintf(status2, "Avoiding asteroid");
			pthread_mutex_unlock(&mux_status);
		}
		else if (check == 1)
		{
			y_vel_ref = 8;
			// update status message
			pthread_mutex_lock(&mux_status);
			sprintf(status2, "Avoiding asteroid");
			pthread_mutex_unlock(&mux_status);
		}
		else
		{
			y_vel_ref = 5; //min( -5, max(-30.0, -45.0/290*y + 89000.0/290)); 
			pthread_mutex_lock(&mux_status);
			sprintf(status2, "Main engine on");
			pthread_mutex_unlock(&mux_status);
		}
		
		theta_ref = 0; 
		err_theta = find_rotation_error(theta_ref, theta);
		
		// regulate theta
		if (err_theta > 0.02 && theta_vel < err_theta*Kp_theta)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.rot_rcs = 1;
			pthread_mutex_unlock(&mux_lem);
		}
		else if (err_theta < -0.02 && theta_vel > err_theta*Kp_theta)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.rot_rcs = -1;
			pthread_mutex_unlock(&mux_lem);
		}
		
		// regulate y_vel	
		if (fabs(err_theta) < 0.2 && y_vel < y_vel_ref)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.main_eng = 1;
			pthread_mutex_unlock(&mux_lem);
		}
		
		// avoid asteroid
		if (check != 0) 
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.lr_rcs = check;
			pthread_mutex_unlock(&mux_lem);
		}
		
		// small adjustments to x_vel with rcs
		else 
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.lr_rcs = -sign(x_vel);
			pthread_mutex_unlock(&mux_lem);
		}
	}
	
	// after asteroid belt burn sideways to get to orbital velocity and correct altitude
	else if (y > ASTBELT_UP + 100 && (!csm_spawned_loc || (fabs(x_vel - CSM_ORBITAL_VEL) > 15 || fabs(y_vel) > 15)))
	{
		// update status message
		pthread_mutex_lock(&mux_status);
		sprintf(status1, "Orbit insertion burn");
		pthread_mutex_unlock(&mux_status);
		
		// if close to apoapsis turn to 90 cw 
		if (y > CSM_ORBITAL_Y - 500 && !csm_spawned_loc)
		{
			// update status message
			pthread_mutex_lock(&mux_status);
			sprintf(status2, "Rotating for burn alignment");
			pthread_mutex_unlock(&mux_status);
			theta_ref = M_PI/2; 
			err_theta = find_rotation_error(theta_ref, theta);
		
			// regulate theta
			if (err_theta > 0.02 && theta_vel < err_theta*Kp_theta)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.rot_rcs = 1;
				pthread_mutex_unlock(&mux_lem);
			}
			else if (err_theta < -0.02 && theta_vel > err_theta*Kp_theta)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.rot_rcs = -1;
				pthread_mutex_unlock(&mux_lem);
			}
			
			// small adjustments to y_apo with rcs
			if (fabs(err_theta) < 0.2 && y_apo > CSM_ORBITAL_Y + 0.1)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.lr_rcs = 1;
				pthread_mutex_unlock(&mux_lem);
			}
		}
		
		// at apoapsis burn to get into orbit
		if (y > CSM_ORBITAL_Y - 20 && !csm_spawned_loc)	
		{
			// update status message
			pthread_mutex_lock(&mux_status);
			sprintf(status2, "Main engine on");
			pthread_mutex_unlock(&mux_status);
			
			theta_ref = M_PI/2; 
			err_theta = find_rotation_error(theta_ref, theta);
			y_err = CSM_ORBITAL_Y - y;
		
			// regulate theta
			if (err_theta > 0.02 && theta_vel < err_theta*Kp_theta)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.rot_rcs = 1;
				pthread_mutex_unlock(&mux_lem);
			}
			else if (err_theta < -0.02 && theta_vel > err_theta*Kp_theta)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.rot_rcs = -1;
				pthread_mutex_unlock(&mux_lem);
			}
			
			// main engine on
			if (fabs(err_theta) < 0.2 && x_vel < CSM_ORBITAL_VEL && fabs(y_err) < 5)
			{
				// update status message
				pthread_mutex_lock(&mux_status);
				sprintf(status2, "Main engine on");
				pthread_mutex_unlock(&mux_status);
				pthread_mutex_lock(&mux_lem); 
				lem_states.main_eng = 1;
				pthread_mutex_unlock(&mux_lem);
			}
			
			// small adjustments to y vel with rcs
			if (fabs(err_theta) < 0.2 && fabs(y_err) < 10)
			{
				if ((y_err > 0 && y_vel > 0.1*y_err) || (y_err < 0 && y_vel > 0.1*y_err) )
				{
					pthread_mutex_lock(&mux_lem); 
					lem_states.lr_rcs = 1;
					pthread_mutex_unlock(&mux_lem);
				}
				else if ((y_err > 0 && y_vel < 0.1*y_err) || (y_err < 0 && y_vel < 0.1*y_err) )
				{
					pthread_mutex_lock(&mux_lem); 
					lem_states.lr_rcs = -1;
					pthread_mutex_unlock(&mux_lem);
				}
			}			
		}
		
		// if apoapsis < csm orbit: set theta_ref=0 and thrust vertically while keeping x_vel=0
		else if (y_apo < CSM_ORBITAL_Y - 1 && x_vel < 20) // && !flag_apo)
		{
			// update status message
			pthread_mutex_lock(&mux_status);
			sprintf(status2, "Raising apoapsis with main engine");
			pthread_mutex_unlock(&mux_status);
			
			theta_ref = 0; 
			err_theta = find_rotation_error(theta_ref, theta);
			
			// regulate theta
			if (err_theta > 0.02 && theta_vel < err_theta*Kp_theta)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.rot_rcs = 1;
				pthread_mutex_unlock(&mux_lem);
			}
			else if (err_theta < -0.02 && theta_vel > err_theta*Kp_theta)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.rot_rcs = -1;
				pthread_mutex_unlock(&mux_lem);
			}
		
			// main engine on
			if (fabs(err_theta) < 0.2) 
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.main_eng = 1;
				pthread_mutex_unlock(&mux_lem);
			}
			
			// small adjustments to x_vel with rcs
			if ((theta > 6.2 || theta < 0.1) && fabs(x_vel) > 0.01)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.lr_rcs = -sign(x_vel);
				pthread_mutex_unlock(&mux_lem);
			}
		}
		
		// if apoapsis is >> csm orbit: reorient and thrust downwards
		else if (y_apo > CSM_ORBITAL_Y + 20 && x_vel < 20) // && !flag_apo)
		{
			// update status message
			pthread_mutex_lock(&mux_status);
			sprintf(status2, "Lowering apoapsis with main engine");
			pthread_mutex_unlock(&mux_status);
			
			theta_ref = M_PI; 
			err_theta = find_rotation_error(theta_ref, theta);
			
			// regulate theta
			if (err_theta > 0.02 && theta_vel < err_theta*Kp_theta)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.rot_rcs = 1;
				pthread_mutex_unlock(&mux_lem);
			}
			else if (err_theta < -0.02 && theta_vel > err_theta*Kp_theta)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.rot_rcs = -1;
				pthread_mutex_unlock(&mux_lem);
			}
		
			// main engine on
			if (fabs(err_theta) < 0.2)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.main_eng = 1;
				pthread_mutex_unlock(&mux_lem);
			}
			
			// small adjustments to x_vel with rcs
			if ((theta > 0.97*M_PI || theta < 1.03*M_PI) && fabs(x_vel) > 0.01)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.lr_rcs = sign(x_vel);
				pthread_mutex_unlock(&mux_lem);
			}
		}
		
		// if apoapsis is > csm orbit: fix it with rcs
		else if (y_apo > CSM_ORBITAL_Y + 0.1 && y < CSM_ORBITAL_Y - 500 && x_vel < 20) // && !flag_apo)
		{
			// update status message
			pthread_mutex_lock(&mux_status);
			sprintf(status2, "Lowering apoapsis with rcs");
			pthread_mutex_unlock(&mux_status);
			
			theta_ref = 0; 
			err_theta = find_rotation_error(theta_ref, theta);
			
			// regulate theta
			if (err_theta > 0.02 && theta_vel < err_theta*Kp_theta)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.rot_rcs = 1;
				pthread_mutex_unlock(&mux_lem);
			}
			else if (err_theta < -0.02 && theta_vel > err_theta*Kp_theta)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.rot_rcs = -1;
				pthread_mutex_unlock(&mux_lem);
			}
		
			// small adjustments to y_apo with rcs
			if ((theta > 6.2 || theta < 0.1))
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.updwn_rcs = -1;
				pthread_mutex_unlock(&mux_lem);
			}
			
			// small adjustments to x_vel with rcs
			if ((theta > 6.2 || theta < 0.1) && fabs(x_vel) > 0.01)
			{
				pthread_mutex_lock(&mux_lem); 
				lem_states.lr_rcs = -sign(x_vel);
				pthread_mutex_unlock(&mux_lem);
			}
		}
	}
	
	// when at correct orbital vel and altitude dock with the csm
	else if(csm_spawned_loc) // && fabs(x_vel - CSM_ORBITAL_VEL) < 15 && fabs(y_vel) < 15)
	{
		// update status message
		pthread_mutex_lock(&mux_status);
		sprintf(status1, "Docking");
		pthread_mutex_unlock(&mux_status);
		
		// adjust relative orientation
		theta_ref = M_PI/2; 
		err_theta = find_rotation_error(theta_ref, theta);
		
		// regulate theta
		if (err_theta > 0.02 && theta_vel < err_theta*Kp_theta)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.rot_rcs = 1;
			pthread_mutex_unlock(&mux_lem);
		}
		else if (err_theta < -0.02 && theta_vel > err_theta*Kp_theta)
		{
			pthread_mutex_lock(&mux_lem); 
			lem_states.rot_rcs = -1;
			pthread_mutex_unlock(&mux_lem);
		}
			
		// adjust y relative positions 	
		y_err = CSM_ORBITAL_Y - y;
		if (fabs(err_theta) < 0.2) // && fabs(y_err) < 10)
		{
			if ((y_err > 0 && y_vel > 0.1*y_err) || (y_err < 0 && y_vel > 0.1*y_err) )
			{
				// update status message
				pthread_mutex_lock(&mux_status);
				sprintf(status2, "Adjusting altitude");
				pthread_mutex_unlock(&mux_status);
				pthread_mutex_lock(&mux_lem); 
				lem_states.lr_rcs = .5;
				pthread_mutex_unlock(&mux_lem);
			}
			else if ((y_err > 0 && y_vel < 0.1*y_err) || (y_err < 0 && y_vel < 0.1*y_err) )
			{
				// update status message
				pthread_mutex_lock(&mux_status);
				sprintf(status2, "Adjusting altitude");
				pthread_mutex_unlock(&mux_status);
				pthread_mutex_lock(&mux_lem); 
				lem_states.lr_rcs = -.5;
				pthread_mutex_unlock(&mux_lem);
			}
		} 
		
		// when y is aligned approach csm 
		x_err = csm_x - x;
		x_rel_vel = x_vel - CSM_ORBITAL_VEL; 
		x_vel_ref = min(5, max(0.2, 4.9/73.5*x_err - 24.5/73.5)); 
		if (fabs(err_theta) < 0.2 && x_err > 2 && fabs(y_err) < 0.1)
		{
			if (x_rel_vel > x_vel_ref)
			{			
				// update status message
				pthread_mutex_lock(&mux_status);
				sprintf(status2, "Approaching CSM");
				pthread_mutex_unlock(&mux_status);
				pthread_mutex_lock(&mux_lem); 
				lem_states.updwn_rcs = -1;
				pthread_mutex_unlock(&mux_lem);
			}
			else if (x_rel_vel < x_vel_ref)
			{
				// update status message
				pthread_mutex_lock(&mux_status);
				sprintf(status2, "Approaching CSM");
				pthread_mutex_unlock(&mux_status);
				pthread_mutex_lock(&mux_lem); 
				lem_states.updwn_rcs = 1;
				pthread_mutex_unlock(&mux_lem);
			}
		}
		
		// if x > x_lem circle around and avoid collision
		if (fabs(err_theta) < 0.2 && x_err < 2)
		{
			// get to a safe vertical distance	
			y_err = CSM_ORBITAL_Y - 15*sign(y_err) - y;
			if (fabs(err_theta) < 0.2)
			{
				if ((y_err > 0 && y_vel > 0.5*y_err) || (y_err < 0 && y_vel > 0.5*y_err) )
				{
					// update status message
					pthread_mutex_lock(&mux_status);
					sprintf(status2, "Adjusting altitude");
					pthread_mutex_unlock(&mux_status);
					pthread_mutex_lock(&mux_lem); 
					lem_states.lr_rcs = 1;
					pthread_mutex_unlock(&mux_lem);
				}
				else if ((y_err > 0 && y_vel < 0.5*y_err) || (y_err < 0 && y_vel < 0.5*y_err) )
				{
					// update status message
					pthread_mutex_lock(&mux_status);
					sprintf(status2, "Adjusting altitude");
					pthread_mutex_unlock(&mux_status);
					pthread_mutex_lock(&mux_lem); 
					lem_states.lr_rcs = -1;
					pthread_mutex_unlock(&mux_lem);
				}
			}
			
			// when safely below the csm move back in front of csm
			x_err = csm_x - x;
			x_rel_vel = x_vel - CSM_ORBITAL_VEL; 
			x_vel_ref = -1; 
			if (fabs(err_theta) < 0.2 && fabs(y_err) < 2)
			{
				if (x_rel_vel > x_vel_ref)
				{			
					// update status message
					pthread_mutex_lock(&mux_status);
					sprintf(status2, "Approaching CSM");
					pthread_mutex_unlock(&mux_status);
					pthread_mutex_lock(&mux_lem); 
					lem_states.updwn_rcs = -1;
					pthread_mutex_unlock(&mux_lem);
				}
				else if (x_rel_vel < x_vel_ref)
				{
					// update status message
					pthread_mutex_lock(&mux_status);
					sprintf(status2, "Approaching CSM");
					pthread_mutex_unlock(&mux_status);
					pthread_mutex_lock(&mux_lem); 
					lem_states.updwn_rcs = 1;
					pthread_mutex_unlock(&mux_lem);
				}
			} 
		}
		
		
			
	}
}

void lem_manual_controls()
// read the pressed key and change the LEM states accordingly
{
	if(key[KEY_W])
	{		
		pthread_mutex_lock(&mux_lem);
		lem_states.updwn_rcs = 1; 
		pthread_mutex_unlock(&mux_lem);
	}
	if(key[KEY_A])	
	{	
		pthread_mutex_lock(&mux_lem);
		lem_states.lr_rcs = -1;
		pthread_mutex_unlock(&mux_lem);
	}		
	if(key[KEY_S])	
	{	
		pthread_mutex_lock(&mux_lem);
		lem_states.updwn_rcs = -1;
		pthread_mutex_unlock(&mux_lem);
	}
	if(key[KEY_D])	
	{	
		pthread_mutex_lock(&mux_lem);
		lem_states.lr_rcs = 1;
		pthread_mutex_unlock(&mux_lem);
	}
		
	if(key[KEY_E])	
	{	
		pthread_mutex_lock(&mux_lem);
		lem_states.rot_rcs = 1;
		pthread_mutex_unlock(&mux_lem);
	}
	if(key[KEY_Q])	
	{	
		pthread_mutex_lock(&mux_lem);
		lem_states.rot_rcs = -1;
		pthread_mutex_unlock(&mux_lem);
	}
	
	if(key[KEY_SPACE])	
	{	
		pthread_mutex_lock(&mux_lem);
		lem_states.main_eng = 1;
		pthread_mutex_unlock(&mux_lem);
	}

	if(key[KEY_R])	
	{	
		pthread_mutex_lock(&mux_lem);
		lem_states.theta_vel = 0;
		pthread_mutex_unlock(&mux_lem);
	}
}

void gui_controls()
// read the pressed key and change the gui states accordingly
{
	if(key[KEY_H])	
	{	
		pthread_mutex_lock(&mux_autop);
		autopilot = !autopilot;
		pthread_mutex_unlock(&mux_autop);
		key[KEY_H] = 0;
	}
	
	if(key[KEY_1])	
	{	
		pthread_mutex_lock(&mux_autop);
		vect_selector = 1; // 1=none, 2=surface, 3=docking
		pthread_mutex_unlock(&mux_autop);
	}
	if(key[KEY_2])	
	{	
		pthread_mutex_lock(&mux_autop);
		vect_selector = 2; // 1=none, 2=surface, 3=docking
		pthread_mutex_unlock(&mux_autop);
	}
	if(key[KEY_3])	
	{	
		pthread_mutex_lock(&mux_autop);
		vect_selector = 3; // 1=none, 2=surface, 3=docking
		pthread_mutex_unlock(&mux_autop);
	}
	
	if(key[KEY_ESC])
	{	
		pthread_mutex_lock(&mux_end);
		end = 1;
		pthread_mutex_unlock(&mux_end);
	}
}

void update_bottom_cp(float x, float y, float theta_deg, float x_vel, float y_vel, float theta_vel, float x_csm, int csm_spawned_loc)
// update the data in the bottom control panel
{
	char	s[30], *s1, *s2;
	bool	autop;
	float 	apoapsis_loc;
	int		vect_sel_loc;
	BITMAP	*cp_buff; 
	
	cp_buff = create_bitmap(XLP, YLP);
	clear_to_color(cp_buff, 0);
	
	// status message
	pthread_mutex_lock(&mux_lem);
	s1 = status1;
	s2 = status2;
	pthread_mutex_unlock(&mux_lem);
	sprintf(s, "STATUS1: ");
	textout_ex(cp_buff, font, s, 10, 0 + 10, 14, BG);

	textout_ex(cp_buff, font, s1, 80, 0 + 10, 14, BG);
	
	sprintf(s, " STATUS2:");
	textout_ex(cp_buff, font, s, 470, 0 + 10, 14, BG);
	
	textout_ex(cp_buff, font, s2, 550, 0 + 10, 14, BG);
	
	// lem states
	sprintf(s, "x: %011.3f", x);
	textout_ex(cp_buff, font, s, 10, 30 + 10, 14, BG);
	
	sprintf(s, "y: %011.3f", y);
	textout_ex(cp_buff, font, s, 10, 30 + 30, 14, BG);
	
	sprintf(s, "theta: %07.3f", theta_deg);
	textout_ex(cp_buff, font, s, 10, 30 + 50, 14, BG);
	
	sprintf(s, "x_vel: %011.3f", x_vel);
	textout_ex(cp_buff, font, s, 140, 30 + 10, 14, BG);
	
	sprintf(s, "y_vel: %011.3f", y_vel);
	textout_ex(cp_buff, font, s, 140, 30 + 30, 14, BG);
	
	sprintf(s, "theta_vel: %07.3f", theta_vel/M_PI*180);
	textout_ex(cp_buff, font, s, 140, 30 + 50, 14, BG);
	
	// autopilot state
	pthread_mutex_lock(&mux_autop);
	autop = autopilot;
	pthread_mutex_unlock(&mux_autop);
	if (autop)
	{
		sprintf(s, "Autopilot: ON");
	}
	else
	{
		sprintf(s, "Autopilot: OFF");
	}
	textout_ex(cp_buff, font, s, 410, 30 + 10, 14, BG);
	
	// apoapsis
	pthread_mutex_lock(&mux_lem);
	apoapsis_loc = apoapsis;
	pthread_mutex_unlock(&mux_lem);
	sprintf(s, "Apoapsis: %011.3f", apoapsis_loc);
	textout_ex(cp_buff, font, s, 410, 30 + 30, 14, BG);
	
	// vector selector
	pthread_mutex_lock(&mux_autop);
	vect_sel_loc = vect_selector; // 1=none, 2=surface, 3=docking
	pthread_mutex_unlock(&mux_autop);
	switch (vect_sel_loc)
	{
	case 2:	
		sprintf(s, "Vector: surface");
		break;
	case 3:
		sprintf(s, "Vector: docking");
		break;
	default:
		sprintf(s, "Vector: none");
		break;
	}
	textout_ex(cp_buff, font, s, 410, 30 + 50, 14, BG);
	
	// csm states
	if (csm_spawned_loc)
	{
		sprintf(s, "relative x: %011.3f", x_csm - x);
		textout_ex(cp_buff, font, s, 710, 30 + 10, 14, BG);
	
		sprintf(s, "relative y: %011.3f", CSM_ORBITAL_Y - y);
		textout_ex(cp_buff, font, s, 710, 30 + 30, 14, BG);
	
		sprintf(s, "relative theta: %07.3f", 0 - theta_deg);
		textout_ex(cp_buff, font, s, 710, 30 + 50, 14, BG);
	
		sprintf(s, "relative x_vel: %011.3f", CSM_ORBITAL_VEL - x_vel);
		textout_ex(cp_buff, font, s, 910, 30 + 10, 14, BG); // +200
	
		sprintf(s, "relative y_vel: %011.3f", 0 - y_vel);
		textout_ex(cp_buff, font, s, 910, 30 + 30, 14, BG);
	
		sprintf(s, "relative theta_vel: %07.3f", (0 - theta_vel)/M_PI*180);
		textout_ex(cp_buff, font, s, 910, 30 + 50, 14, BG);
	}
	
	// copy buffer to the screen
	blit(cp_buff, screen, 0, 0, 0, YWIN, cp_buff->w, cp_buff->h);
	
	// destroy buffer to save memory
	destroy_bitmap(cp_buff);
}

void update_right_cp()
// update the controls in the right control panel
{
	char	s[30];
	
	sprintf(s, "CONTROLS: ");
	textout_ex(screen, font, s, XWIN + 10, 10, 14, BG);
			
	sprintf(s, "Translation with RCS: ");
	textout_ex(screen, font, s, XWIN + 30, 30, 14, BG);
	sprintf(s, "W-A-S-D = up-left-down-right");
	textout_ex(screen, font, s, XWIN + 40, 40, 14, BG);
	
	sprintf(s, "Rotation with RCS: ");
	textout_ex(screen, font, s, XWIN + 30, 60, 14, BG);
	sprintf(s, "E-Q = clockwise-conter clockwise");
	textout_ex(screen, font, s, XWIN + 40, 70, 14, BG);
	
	sprintf(s, "Main engine ON/OFF: ");
	textout_ex(screen, font, s, XWIN + 30, 90, 14, BG);
	sprintf(s, "SPACE (toggle)");
	textout_ex(screen, font, s, XWIN + 40, 100, 14, BG);
	
	sprintf(s, "Autopilot ON/OFF:");
	textout_ex(screen, font, s, XWIN + 30, 120, 14, BG);
	sprintf(s, "H (toggle)");
	textout_ex(screen, font, s, XWIN + 40, 130, 14, BG);
	
	sprintf(s, "Cancel angular velocity:");
	textout_ex(screen, font, s, XWIN + 30, 150, 14, BG);
	sprintf(s, "R");
	textout_ex(screen, font, s, XWIN + 40, 160, 14, BG);
	
	sprintf(s, "Select vector:");
	textout_ex(screen, font, s, XWIN + 30, 180, 14, BG);
	sprintf(s, "1 = none");
	textout_ex(screen, font, s, XWIN + 40, 190, 14, BG);
	sprintf(s, "2 = surface");
	textout_ex(screen, font, s, XWIN + 40, 200, 14, BG);
	sprintf(s, "3 = docking");
	textout_ex(screen, font, s, XWIN + 40, 210, 14, BG);
	
	sprintf(s, "ALTITUDE MAP: ");
	textout_ex(screen, font, s, XWIN + 10, 300, 14, BG);
}

void update_minimap()
// update the altitude map in the right control panel
{
	BITMAP *map, *lem;
	map = load_bitmap("bitmaps/minimap.bmp", NULL);
	lem = load_bitmap("bitmaps/lem_map.bmp", NULL);
	
	float y;
	
	// read lem altitude
	pthread_mutex_lock(&mux_lem);
	y = lem_states.y;
	pthread_mutex_unlock(&mux_lem);
	
	stretch_sprite(map, lem, 60 - 10, -0.1*y + 390 - 10, 20, 20);
	
	blit(map, screen, 0, 0, XWIN + 30, 320, map->w, map->h);
	
	// destroy bitmaps to save memory
	destroy_bitmap(map);
	destroy_bitmap(lem);	
}

int add_ast(int n_ast_loc)
// set initial values for LEM states
{	
	float	lem_x, lem_y, lem_x_vel, lem_y_vel;
	
	// read lem states
	pthread_mutex_lock(&mux_lem); 
	lem_x = lem_states.x;
	lem_y = lem_states.y;
	lem_x_vel = lem_states.x_vel;
	lem_y_vel = lem_states.y_vel;
	pthread_mutex_unlock(&mux_lem);
	
	// create new asteroid
	pthread_mutex_lock(&mux_ast);
	ast_states[n_ast_loc].x = lem_x - randf(0, XWIN/10); 
	ast_states[n_ast_loc].y = lem_y + sign(lem_y_vel)*YWIN/20;
	ast_states[n_ast_loc].theta = randf(0, 2*M_PI);
	ast_states[n_ast_loc].x_vel = lem_x_vel + randf(2, 14);
	ast_states[n_ast_loc].y_vel = 0;
	ast_states[n_ast_loc].theta_vel = randf(-1.2, +1.2);
	n_ast = n_ast_loc + 1;
	pthread_mutex_unlock(&mux_ast);
	
	return n_ast_loc + 1;
	
}

int check_collision_path(float vref, int descent)
// check if the lem is on a collision path with any asteroid: 
// returns 0 if not, -1 if right side, 1 if left side
{
	int		n_ast_loc, i, i_lower, safety = 5;
	float	x[MAX_AST], x_new[MAX_AST], y[MAX_AST], x_vel[MAX_AST], dt[MAX_AST];
	float	lem_x, lem_y, lem_x_vel, lem_y_vel, lem_x_new;
	
	// read asteroids data 
	pthread_mutex_lock(&mux_ast);
	n_ast_loc = n_ast;
	for (i = 0; i < n_ast_loc; i++)
	{
		x[i] = ast_states[i].x;
		y[i] = ast_states[i].y;
		x_vel[i] = ast_states[i].x_vel;
	}
	pthread_mutex_unlock(&mux_ast);
	
	// read lem data
	pthread_mutex_lock(&mux_lem); 
	lem_x = lem_states.x;
	lem_y = lem_states.y;
	lem_x_vel = 0; // lem_states.x_vel; 
	lem_y_vel = lem_states.y_vel;
	pthread_mutex_unlock(&mux_lem);
	
	vref = fabs(vref);
	
	// if descending 
	if (descent)
	{
		for (i = 0; i < n_ast_loc; i++)
		{
			if (lem_y > y[i] + 10)
			{
				// compute time to get to the y of every lower asteroid
				dt[i] = fabs(lem_y - 3.6 - (y[i] + 2))/(vref); 
				// compute future x of asteroids after dt
				x_new[i] = x_vel[i]*dt[i] + x[i];
				// compute future x of the lem after dt
				lem_x_new = lem_x_vel*dt[i] + lem_x;
			
				// find future collisions
				if (lem_x_new - x_new[i] < (32 + 20 +(x_vel[i] - lem_x_vel)/(vref)*(72 + 20) + safety)/10 && lem_x_new - x_new[i] > -(32 + 20 + safety)/10)
				{
					if ((lem_x_new - x_new[i])*(x_vel[i] - lem_x_vel)/(vref) > (36 + 20)/10)
					{
						return 1;
					}
					else
					{
						return -1;
					}
				}
			}
		}
	}
	
	// if ascending 
	else
	{
		for (i = 0; i < n_ast_loc; i++)
		{
			if (lem_y < y[i] - 10)
			{
				// compute time to get to the y of every lower asteroid
				dt[i] = fabs(y[i] - 2 - (lem_y + 3.6))/(vref); 
				// compute future x of asteroids after dt
				x_new[i] = x_vel[i]*dt[i] + x[i];
				// compute future x of the lem after dt
				lem_x_new = lem_x_vel*dt[i] + lem_x;
			
				// find future collisions
				if (lem_x_new - x_new[i] < (32 + 20 +(x_vel[i] - lem_x_vel)/(vref)*(72 + 20) + safety)/10 && lem_x_new - x_new[i] > -(32 + 20 + safety)/10)
				{
					if ((lem_x_new - x_new[i])*(x_vel[i] - lem_x_vel)/(vref) > (36 + 20)/10)
					{
						return 1;
					}
					else
					{
						return -1;
					}
				}
			}
		}
	}
	
	return 0;
}





