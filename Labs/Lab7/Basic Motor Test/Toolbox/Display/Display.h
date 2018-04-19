#ifndef _DISPLAY_H
#define _DISPLAY_H

/*****************************************
 * Function that draws a grid on the LCD
 * for easier readout of whatever is plot
 *****************************************/
	void draw_grid(){
		for(int i = 0; i < 65; i++)
		{
			nxtSetPixel(50, i);
			int grid5 = (i - 32) % 5;
			int grid10 = (i - 32) % 10;
			if(!grid5 && grid10)
			{
				for(int j = -2; j < 3; j++)
				{
					nxtSetPixel(50 + j, i);
				}
			}
			else if(!grid10)
			{
				for(int j = -4; j < 5; j++)
				{
					nxtSetPixel(50 + j, i);
				}
			}
		}
		for(int i = 0; i < 101; i++)
		{
			nxtSetPixel(i, 32);
			int grid5 = (i - 100) % 5;
			int grid10 = (i - 100) % 10;
			if(!grid5 && grid10)
			{
				for(int j = -2; j < 3; j++)
				{
					nxtSetPixel(i, 32 + j);
				}
			}
			else if(!grid10)
			{
				for(int j = -4; j < 5; j++)
				{
					nxtSetPixel(i, 32 + j);
				}
			}
		}
	}

#endif // _DISPLAY_H
