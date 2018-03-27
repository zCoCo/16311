#ifndef _UNITS_H
#define _UNITS_H

// Standard Unit System used Throughout Toolbox:
// Time:        Seconds (small exception in some deltas)
// Distance:    Meters
// Angle:       Radians

// CONVERSIONS:
  // Converts values in given unit system to standard units system through
  // multiplication.
  #define DEG 0.017453 /* ex. 180*DEG = 3.14159 */
  #define INCH 0.0254 /* ex. 1*INCH = 0.0254 */
  #define FT 0.3048 /* ex. 1*FT = 0.3048 */

#endif // _UNITS_H
