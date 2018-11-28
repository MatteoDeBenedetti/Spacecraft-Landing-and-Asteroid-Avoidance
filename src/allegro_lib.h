/*----------------------------------------------------------------------------*/
// ALLEGRO LIBRARY --- HEADER
// Useful functions for handling allegro
/*----------------------------------------------------------------------------*/

#ifndef ALLEGRO_LIB_H
#define ALLEGRO_LIB_H


/*----------------------------------------------------------------------------*/

#include <time.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <allegro.h>


/*----------------------------------------------------------------------------*/
/*Constants------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*
#define XSCREEN 1920 // screen width
#define YSCREEN 1080 // screen height
#define XWIN 1366 // scenario window width
#define YWIN 768 // scenario window height
#define XLP 1366 // lower control panel width (=XWIN)
#define YLP 312 // lower control panel height (=YSCREEN-YWIN)
#define XRP 554 // right control panel width (=XSCREEN-XWIN)
#define YRP 1080 // right control panelheight (=YSCREEN)
*/

#define XSCREEN 1600 // screen width
#define YSCREEN 900 // screen height
#define XWIN 1266 // scenario window width
#define YWIN 768 // scenario window height
#define XLP 1366 // lower control panel width (=XWIN)
#define YLP 132 // lower control panel height (=YSCREEN-YWIN)
#define XRP 334 // right control panel width (=XSCREEN-XWIN)
#define YRP 900 // right control panelheight (=YSCREEN)

/*
#define XSCREEN 1366 // screen width
#define YSCREEN 768 // screen height
#define XWIN 1116 // scenario window width
#define YWIN 618 // scenario window height
#define XLP 1166 // lower control panel width (=XWIN)
#define YLP 150 // lower control panel height (=YSCREEN-YWIN)
#define XRP 250 // right control panel width (=XSCREEN-XWIN)
#define YRP 768 // right control panelheight (=YSCREEN)
*/

#define BG 0 // background color


/*----------------------------------------------------------------------------*/
/*Structures------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
/*Functions Prototipes--------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void get_keycode(char *scan, char *ascii);
void get_string(char *str, int x, int y, int c, int b);
void init_gui();
char get_scancode();







/*----------------------------------------------------------------------------*/

#endif

