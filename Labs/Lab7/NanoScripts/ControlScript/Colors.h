#ifndef COLORS_H
#define COLORS_H

int DARK[3] = {0, 0, 0};

int RED[3] = {1, 0, 0};
int GREEN[3] = {0, 255, 0};
int BLUE[3] = {0, 0, 255};

int WHITE[3] = {1, 255, 200};

int *COMMAND_COLORS[5] = {DARK, RED, GREEN, BLUE, WHITE};

bool currRed = 0;
int currGreen = 0;
int currBlue = 0;
void color(bool r, int g, int b){
  digitalWrite(LED_R, r ? HIGH : LOW);
  analogWrite(LED_G, g);
  analogWrite(LED_B, b);
  currRed = r; currGreen = g; currBlue = b;
}

void color(int* c){
  color(c[0], c[1], c[2]);
}

#endif //COLORS_H
