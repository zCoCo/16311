/**********************************************
 * Function that judges if two floats are equal
 **********************************************/
 bool equal(float a, float b) {
   float epsilon = 1;
   if (abs(a-b) < epsilon) {
     return true;
   } else {
     return false;
   }
 }
