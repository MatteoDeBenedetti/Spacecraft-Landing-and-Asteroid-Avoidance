/*----------------------------------------------------------------------------*/
// ALLEGRO LIBRARY --- SOURCE
// Useful functions for handling allegro
/*----------------------------------------------------------------------------*/

#define _GNU_SOURCE
#include <time.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <allegro.h>

#include "allegro_lib.h"


/*----------------------------------------------------------------------------*/
/*Functions Definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void get_keycode(char *scan, char *ascii)
// wait for a key pressed and saves in arg1 the scan code and arg2 the ascii code
{
	int k;
	
	k = readkey();
	*ascii = k;
	*scan = k >> 8;
}

void get_string(char *str, int x, int y, int c, int b)
// read characters and print in x=arg2,y=arg3 color arg4, background arg5
{
	char ascii, scan, s[2];
	int i = 0;
	
	do
	{
		get_keycode(&scan, &ascii);
		
		if (scan != KEY_ENTER)
		{
			s[0] = ascii;
			s[1] = '\0';
			textout_ex(screen, font, s, x, y, c, b);
			x += 8;
			str[i++] = ascii;			
		}
	} while (scan != KEY_ENTER);
	
	str[i] = '\0';
}

void init_gui() 
// starts allegro and the screen
{
	allegro_init();
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, XSCREEN, YSCREEN, 0, 0);	
	clear_to_color(screen, BG);
		
	install_keyboard();
	
	srand(time(NULL));
}

char get_scancode()
// read the scancode of the key. not blocking
{
	if (keypressed())
	{
		return readkey() >> 8;
	}
	else 
	{
		return 0;
	}
}











