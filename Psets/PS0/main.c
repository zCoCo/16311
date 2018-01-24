/* @file p1.h
 *
 * 16-311 Introduction to Robotics: Homework 0
 *
 * @author Hannah
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "p1.h"

int main(void)
{
    char buff[80];
    int done = 0;
    while(!done) {
        fprintf(stdout, "Please type a vector or matrix.\nPlease type 'HELP' for help and 'END' to stop the session.\n");

        gets(buff); /* Careful, this is not a very safe method */

        // parse_input it and continue
        done = Parse(buff); /* if input is 'END', will return 1 */
    }
    return 0;
}
