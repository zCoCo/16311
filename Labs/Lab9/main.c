/****
 * 16-311 Lab-9
 * Basic Coordination Script for SCARA (Revolute-Revolute) Robot Arm, Oriented
 * Vertically (against gravity, requires compensation).
 * List of Target Points in X-Y Plane are Given. Visited by Navigating from Current
 * Position to Voronoi Roadmap (generated in accompanying MatLab files), to
 * Roadmap Vertex Closest to the Target Position, then to the Target Position.
 * Coordinate Spaces and Arm Orientation and Geometry are Further Outlined in
 * Configuration Space MatLab file, ConfigurationSpacePlanner.m .
****/

#define JointA motorA
#define JointB motorB

#define NUM_TARGETS 3
// List of Targets in X-Y Space, units inches.
float targs[][] = {
  {3,3},
  {4,4},
  {-3,3}
};

/**** -- PATH-PLANNING -- ****/

// Define Vertices and Connectivity of Voronoi Roadmap:
#define NUM_VERTICES 16
// Vertices in ThetaA-ThetaB Space, units deg:
float vrm[][] = {
{1,1},
{1,1},
{1,1},
{1,1},
{1,1},
{1,1},
{1,1},
{1,1},
{1,1},
{1,1},
{1,1},
{1,1},
{1,1},
{1,1},
{1,1},
{1,1},
};
// Connectivity (Adjacency) Matrix of Roadmap Vertices (down is from, across is to):
bool MC[][] = {
  {1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,0},
  {0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,1,0,1,1,0,1,0,0,0,0},
  {0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,1,0,0,1,1,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0},
  {0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1},
}

#define sq(a) ((a)*(a))

// Returns the Index of the Roadmap Vertex Closest to the Given Position (ta,tb)
int findNearestVertex(float ta, float tb){
  int closest = 0; // Index of Closest Index
  float dist = Inf;

  // Loop through Every Vertex, If Connected to Vertex idx, Check if Closer.
  for(int i=0; i<NUM_VERTICES; i++){
    d = sqrt(sq(vrm[i][0] - ta) + sq(vrm[i][1] - tb));
    dist = (d<dist) ? d : dist;
    closest = i;
  }

 return i;
} // #findNearestVertex

// Returns the Index of the Roadmap Vertex Adjacent to the Given Vertex with
// Index idx which is Closest to the Given Target Position, (ta, tb).
int findBestAdjacentVertex(int idx, float ta, float tb){
  int closest = idx; // Index of Closest Index
  float dist = sqrt(sq(vrm[idx][0] - ta) + sq(vrm[idx][1] - tb));

  // Loop through Every Vertex, If Connected to Vertex idx, Check if Closer.
  for(int i=0; i<NUM_VERTICES; i++){
    if(MC[idx][i]){
      d = sqrt(sq(vrm[i][0] - ta) + sq(vrm[i][1] - tb));
      dist = (d<dist) ? d : dist;
      closest = idx;
    }
  }

 return i;
} // #findBestAdjacentVertex

// Simple (quick) Depth First Search of Roadmap Vertices for Path from the
// Given Start Vertex with Index, is, to the Given Target Vertex, it.
int* DFS(int is, int it){
  int path[NUM_VERTICES]; // <- initialize (path will necessarily not contain
                         //  more stops than number of vertices.)
  path[0] = is; // Path Starts with Starting Vertex
  int path_length = 1; // Number of Vertices in Path
  int visited[NUM_VERTICES]; // List of Vertices Visited in Search
  int num_visited = 0; // Number of Vertices Visited

  int no_path = 0; // Whether No Path is Possible (Should Never Happen for Real
                  //  (complete, continuous, fully-connected) Roadmap).
  while(path[path_length-1] != it && !no_path){
    // Loop through Every Vertex, If Connected to Current Position and Not Visited,
    // Visit.
    int no_visit = 1; // Whether any No Vertices were Visited (needs back-track)
    for(int i=0; i<NUM_VERTICES; i++){
      if(MC[path[path_length-1]][i]){
        int vis = 0; // Whether this Vertex has been Visited Yet.
        int j=0;
        while(j<num_visited && !vis){ // Allow Early Termination
          vis = (i==visited[j]);
         j++;
        }

        if(!vis){
          // Visit Node, add to path (preliminarily):
          path[path_length] = i;
          path_length++;
          visited[num_visited] = i;
          num_visited++;
          no_visit = 0;
          break;
        } // !vis?
      }
    }
    if(no_visit){
      // No Visitable Nodes Adjacent to this Node. Backtrack.
      path_length--;
      if(path_length < 0){ // No Nodes remain Visitable from Starting Node.
                          //  All Branches Explored. No Path Possible.
        no_path = 1;
      } // path_length?
    } // no_visit?
  } // loop: path

  if(no_path){
    path[0] = -1 // Indicate No Path was Found
  }

  return path;
} // #DFS

/**** -- KINEMATICS -- ****/

float atan2(float y, float x){ // Result in radians.
  float th;
  if(x>0){
    th = atan(y/x);
  } else if(x<0 && y>=0){
    th = atan(y/x) + PI;
  } else if(x<0 && y<0){
    th = atan(y/x) - PI;
  } else if(x==0 && y>0){
    th = PI/2.0;
  } else if(x==0 && y<0){
    th = -PI/2.0;
  } else{ // x=0, y=0
    th = 0;
  }
 return th;
}

#define sgn(x) ((x) / abs(x))

// Geometry:
#define L1 3.75
#define L2 2.50

// Computes the SCARA IK to returns an Array representing the Arm Configuration
// Necessary for the End-Effector to be Positioned at the Given Position (x,y).
// Results in Deg.
float* computeIK(float x, float y){
  float cfg[2];
  cfg[1] = acos( (sq(x) + sq(y) - sq(L1) - sq(L2)) / (2.0 * L1 * L2) );
  cfg[0] = atan2(y,x) - asin( (L2 * sin(cfg[1])) / sqrt(sq(x) + sq(y)) ) - PI/2.0;
  cfg[0] = cfg[0] * 180.0 / PI; // Convert to Deg.
  cfg[1] = cfg[1] * 180.0 / PI;
} // #computeIK

float init_pos_TH[] = {-90.0, 0.0};

// Returns the Estimated Angle of Joint A
float JointAngleA(){
  nMotorEncoder[JointA] + init_pos_TH[0];
} // #JointAngleA
// Returns the Estimated Angle of Joint A
float JointAngleB(){
  nMotorEncoder[JointB] + init_pos_TH[1];
} // #JointAngleB

// Target Net Translation Speed through ThetaA-ThetaB Space:
#define TRANSLATION_SPEED 20.0
#define Kp 1.0
// Go To the Given Arm Position / Configuration in ThetaA-ThetaB Space.
void goTo_ang(float tha, float thb){
  errA = abs(tha - JointAngleA());
  errB = abs(thb - JointAngleB());

  eps = 1.25; // Allowable Angle Tolerance (resolution of motor)
  while(errA > eps && errB > eps){
    errA = tha - JointAngleA();
    errB = thb - JointAngleB();
    float err = sqrt( sq(errA) + sq(errB) );
    float spd = TRANSLATION_SPEED * Kp * err;

    float phi = atan2(tha, thb); // Direction of Travel through ThetaA-ThetaB Space
    motor[JointA] = ((float) spd * sin(phi) * sgn(errA));
    motor[JointB] = ((float) spd * cos(phi) * sgn(errB));
  }

  motor[JointA] = 0; // stop
  motor[JointB] = 0;
} // #goTo_ang

/**** -- OPERATIONS -- ****/

void init(){
  // Initialize Motors:
  bFloatDuringInactiveMotorPWM = false;

  nMotorPIDSpeedCtrl[JointA] = mtrSpeedReg;
  nMotorPIDSpeedCtrl[JointB] = mtrSpeedReg;
  nMaxRegulatedSpeedNxt = 700;
  nPidUpdateInterval = 2;

  motor[JointA] = 0;
  motor[JointB] = 0;

  nMotorEncoder[JointA] = 0;
  nMotorEncoder[JointB] = 0;
} // #init

task main(){

  init();

  for(int i=0; i<NUM_TARGETS; i++){
    // Get Target:
    float* targ_AB = computeIK(targs[i][0], targs[i][1]); // Target Position in ThA-ThB Space

    // Get Nodes for Path through Roadmap:
    int is = findNearestVertex(JointAngleA(), JointAngleB());
    int it = findNearestVertex(targ_AB[0], targ_AB[1]);

    // Find Path Through Roadmap
    int* path = DFS(is,it);

    if(path[0] != -1){ // Path was Found
      // Go To Each Position in the Roadmap Path. End on Target.
      int idx = -1; // Index of Current Vertex
      int = 0;
      while(idx != it){
        idx = path[i];
        goTo_ang(vrm[idx][0], vrm[idx][1]);
       i++;
      }
      goTo_ang(targ_AB[0], targ_AB[1]);
    } // path[0]?
  } // loop: targets
} // #main
