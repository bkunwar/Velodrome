// ----------------------------------------------------------------------------------------------------
// ELLIPTICAL HYPERBOLIC PARABOLOID ANALYSIS SOFTWARE
// ----------------------------------------------------------------------------------------------------
// Uses Alistair Day's Dynamic Relaxation technique, a simplified version of the Verlet algorithm
// Adapted from Chris William's SimpleDynamicRelaxation program
// by Bharat Kunwar
// ----------------------------------------------------------------------------------------------------

// import processing.opengl.*;

// Primary variables
int 
m = 30, // 50
dm = 5, 
Variable        = 2, // 1 = m varies, 2 = CableT0 varies, 3 = TrussThickness varies
ThisTest        = 0,
WriteInterval   = 1000, 
DisplayInterval = 1,//WriteInterval, 
ReleaseTrigger  = 0, 
ClosingTrigger   = 10000;

// The following are required for operation
int
n, 
o, 
Iteration, 
CentralNode, 
LastNode, 
LastMember;
float
WidestInnerRadius,
MaxTrussT, 
MaxTrussC, 
MaxCableT, 
MaxCableC, 
MinTrussT, 
MinTrussC, 
MinCableT, 
MinCableC, 
dCableT, 
dCableC, 
dTrussT, 
dTrussC, 
Area, 
Circumference, 
NetScale, 
Theta, 
ScaleFactor, 
TotalTrussLength, 
TotalCableLength, 
InitialRotation, 
OuterRingScale;

// Measurement pointers
float
dX, 
dY, 
dC, 
eX, 
eY, 
eZ, 
X0, 
Y0, 
C0;

// Member attributes in mm
float 
TrussRadius    = 40.0, 
TrussThickness = 10.0, 
dTT            = 1.0, 
CableRadius    = 10.0, 
InnerTrussLength = 2500.0;

// Member attributes in N/mm2
float
TrussE = 200 * pow(10, 3), 
CableE = 200 * pow(10, 3),
CableST = 500,
CableSC = 0,
TrussST = 500,
TrussSC = 350;

// The following are defined in the "setup" subroutine
float
TrussA, 
CableA, 
CableEA, 
TrussEA, 
CableT0, 
TrussT0,
Load, 
dP;

// Dimensionless scaling factors
float
xScale = 1.0, 
yScale = 1.0, //0.6
zScale = 0.15; //0.15

// High damping value leads to faster convergence, 0.98 is the optimum
float
Damping = 0.98;

// At what velocity to assume equilibrium in mm/s
float
EquilibriumVelocity = 0.1;
// What colour setting to use?
char Colour = 'F';  // A=FirstMember, F=MemberForce

// Boolean settings
boolean 
Relaxable           = true,
FileOutputEnabled   = false,
TakeScreenshot      = true,
ShowSupports        = true,
Failure             = false,
Quit                = false,
UnstrainOnRelease   = false, 
EdgeTangentCable    = false, 
CableCompressible   = false, 
OutputComplete      = false, 
SummaryInitialized  = false, 
NewCycle            = false, 
EquilibriumReached  = false, 
Released            = false; 

// Output file settings
String SummaryFileName = "BAT", 
FileName = "VAR", 
FileExtension = ".csv", 
DataFolder = "data/", 
CurrentSummaryFileName, 
CurrentFileName;

// Pointers for the files
PrintWriter 
SummaryOutput, 
Output;

// Colour settings
int 
ColourContrast = 255, 
TrussColour = 255, 
CableColour = 0;

// Variables for text output
float 
PTextX = 10.0, 
PTextY = 10.0, 
DTextY = 15.0, 
FontSize = 12.0;
String
FontType = "Courier New";

// Variables for input and output control
float
MyMouseX = 0.0, 
MyMouseY = 0.0, 
MyMouseXpressed = 0.0, 
MyMouseYpressed = 0.0, 
xTrans, 
yTrans, 
zRot, 
xRot;

// Node attribute matrices
float []
K; // Node Stiffness
float [][]
X, // Coordinate
F, // Force
V; // Velocity

int [][] 
Fix;

// Member attribute matrices
float []
EA, // Young's Modulus times the Area
T0, // Pretension
L0, // Original Length
T;  // Tension
int [][] 
End; // Nodes corresponding to the two ends of a given members

// ----------------------------------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------------------------------

void setup() {  
  ResetVariables();

  // Amount of prestress in N
  dP = 0.0002 * CableEA; // 0.2% = 0.002

  if (Variable != 2 || !FileOutputEnabled) CableT0 = dP * 10;
  TrussT0 = 0;
  Load    = CableT0 * 1000;

  ConstructNodes();
  ConstructMembers();

  ResetNodeStiffness(0, LastMember);

  CalculateTotalLength();
  ResetPointers();

  InitiateSummaryOutput();
  InitiateOutput();
}

// ----------------------------------------------------------------------------------------------------
// CONSTRUCTORS
// ----------------------------------------------------------------------------------------------------

// Reset all the variable dependent statics
void ResetVariables() {
  // size(1200,750,OPENGL);
  // size(int(0.9*float(screen.width)),int(0.9*float(screen.height)),P3D);
  size(1200, 750, P3D);
  smooth();
  textFont(createFont(FontType, FontSize));
  textMode(SCREEN);

  println ("Initialising variables");  

  // m over 2 rounded down which gives the number of extra cables on each side
  n = int(m / 2);
  // m minus 1 over 2 rounded down
  o = int((m - 1) / 2);
  // Counting from zero
  LastNode = m * (m + 8) + (4 * n) * (m - n - 1) - 1;
  // Counting from zero
  LastMember = 2 * m * (m + 9) + 4 * o * (m - o) + (4 * n) * (m - n - 1) - 1;

  // Rotate by Theta/2 and then 270 degrees 5*PI/4+PI/(4*m)
  InitialRotation = PI / 4 * (5 + 1 / float(m));
  // Renders the internal truss members the same length as the external members      
  OuterRingScale = PI * sqrt(3) / 4.0 / float(m) + 1;
  // Determine the pointer for the central node
  CentralNode = int(float(m) * (8 + float(m) / 2) + (float(m) + 1) % 2 * float(m) / 2);

  X = new float[LastNode + 1][3];
  F = new float[LastNode + 1][3];
  V = new float[LastNode + 1][3];
  K = new float[LastNode + 1];
  End = new int[LastMember + 1][2];  
  Fix = new int[LastNode + 1][3];  

  EA = new float[LastMember + 1];
  T0 = new float[LastMember + 1];
  L0 = new float[LastMember + 1];
  T = new float[LastMember + 1];

  // in mm2
  TrussA = PI*(pow(TrussRadius, 2)-pow(TrussRadius-TrussThickness, 2));
  CableA = PI*(pow(CableRadius, 2));

  // in N
  CableEA = CableE*CableA;
  TrussEA = TrussE*TrussA;

  NetScale = 1 / sqrt(2) * (1 - PI / 4 / float(m)); // To make the net fit within the truss ring
  WidestInnerRadius = InnerTrussLength * 2 * float(m) / PI;
  ScaleFactor = float(height) / (3.0 * WidestInnerRadius);

  Area = PI * pow(WidestInnerRadius, 2) * xScale * yScale;
  Circumference = PI * WidestInnerRadius * (3 * (xScale + yScale) - sqrt((3 * xScale + yScale) * (xScale + 3 * yScale)));

  zRot = -PI/3;
  xRot = -PI/3;
  xTrans = 0.0;
  yTrans = 0.0;  

  Iteration = 0;
  Released = false;
}

// Construct the nodes of the structure
void ConstructNodes() {
  int Node = -1;

  // Construct the nodes in the Internal Ring
  for (int i = 0; i < 4 * m; i++) {
    Node++;
    Theta = 2 * PI * i / (4 * m) + InitialRotation;
    X[Node][0] = xScale * WidestInnerRadius * cos(Theta); //Position in x axis
    X[Node][1] = yScale * WidestInnerRadius * sin(Theta); //Position in y axis
    if ((i + 1) % 1 == 0) for (int xyz = 0; xyz <= 2; xyz++) Fix[Node][xyz] = 1;
  }

  // Construct the nodes in the External Ring
  for (int i = 0; i < 4 * m; i++) {
    Node++;
    Theta = 2 * PI * i / (4 * m) + PI / (4 * m) + InitialRotation;
    X[Node][0] = xScale * OuterRingScale * WidestInnerRadius * cos(Theta); //Position in x axis
    X[Node][1] = yScale * OuterRingScale * WidestInnerRadius * sin(Theta); //Position in y axis
    if ((i + 1) % 1 == 0) for (int xyz = 0; xyz <= 2; xyz++) Fix[Node][xyz] = 1;
  }

  // Construct the nodes in the Central Cable Net
  for (int j = 0; j < m; j++) {
    for (int i = 0; i < m; i++) {
      Node++;
      X[Node][0] = X[i][0];     //Position in x axis
      X[Node][1] = X[m + j][1]; //Position in y axis
    }
  }

  // Construct the nodes in the Periphery Cable Net
  for (int k = 0; k < 4; k++) {
    for (int j = 1; j <= n; j++) {
      for (int i = j; i < m - j; i++) {
        Node++;
        int xNode = (i + m * k) * ((k + 1) % 2) + (j - 1 + m * k) * (k % 2);
        int yNode = (i + m * k) * (k % 2) + (j - 1 + m * k) * ((k + 1) % 2);
        X[Node][0] = X[xNode][0]; //Position in x axis
        X[Node][1] = X[yNode][1]; //Position in y axis
      }
    }
  }

  println("Check on last node number: Should be " + Node + " and is " + LastNode);
  LastNode = Node;

  // Construct the nodes in the z-axis
  for (Node = 0; Node <= LastNode; Node++) {
    X[Node][2] = zScale * (pow(X[Node][0]/pow(xScale, 1), 2) - pow(X[Node][1]/pow(yScale, 1), 2)) / WidestInnerRadius; //Position in z axis
  }
}

// Construct the members of the structure
void ConstructMembers() {
  int Member = -1;

  // Internal Ring
  for (int i = 0; i < 4 * m; i++) {
    Member++;
    End[Member][0] = i;
    if ((i + 1) == 4 * m) End[Member][1] = 0;
    else End[Member][1] = i + 1;
  }

  // External Ring
  for (int i = 0; i < 4 * m; i++) {
    Member++;
    End[Member][0] = 4 * m + i;
    if ((i + 1) == 4 * m) End[Member][1] = 4 * m;
    else End[Member][1] = 4 * m + i + 1;
  }

  // Truss
  for (int i = 0; i < 4 * m; i++) {
    // LHS Member
    Member++;
    End[Member][0] = i;
    End[Member][1] = i + 4 * m;

    // RHS Member
    Member++;
    if ((i + 1) == 4 * m) End[Member][1] = 0;
    else End[Member][0] = i + 1;
    End[Member][1] = i + 4 * m;
  }


  // Edge Tangent Cables
  if (EdgeTangentCable) {
    for (int k = 0; k < 4; k++) {
      int ThisSide = m * (m + 8) + k * n * (m - n - 1) - 1;
      for (int j = 1; j <= o; j++) {
        Member++;
        End[Member][0] = k * m + j - 1;
        for (int i = j; i < m - j; i++) {
          ThisSide++;
          End[Member][1] = ThisSide;
          Member++;
          End[Member][0] = End[Member - 1][1];
        }
        End[Member][1] = k * m + m - j;
      }
    }
  }

  // Edge Cables
  for (int j = 0; j < m; j++) {
    for (int i = 0; i < m; i++) {
      int ThisSide = m * (m + 8) - 1;
      if (j == 0) {
        //Top
        Member++;
        if (i > 0 && i < m - 1 && EdgeTangentCable) ThisSide += 0;
        else ThisSide = 0; //Edge
        End[Member][0] = ThisSide + i; //Edge            
        End[Member][1] = 8 * m + (m * j) + i; //Middle
      }
      if (j == (m - 1)) {
        //Bottom         
        if (i > 0 && i < m - 1 && EdgeTangentCable) ThisSide += 2 * n * (m - n - 1); // Edge
        else ThisSide = 2 * m; // Corner
        Member++;
        End[Member][0] = ThisSide + m - 1 - i;
        End[Member][1] = 8 * m + (m * j) + i; // Net
      }
      if (i == 0) {
        //Left
        Member++;
        if (j > 0 && j < m - 1 && EdgeTangentCable) ThisSide += 3 * n * (m - n - 1);
        else ThisSide = 3 * m; //Edge   
        End[Member][0] = ThisSide + m - 1 - j; //Edge                 
        End[Member][1] = 8 * m + (m * j); //Middle
      }
      if (i == (m - 1)) {
        //Right
        Member++;
        if (j > 0 && j < m - 1 && EdgeTangentCable) ThisSide += n * (m - n - 1);
        else ThisSide = m;
        End[Member][0] = ThisSide + j; //Edge                               
        End[Member][1] = 8 * m + m * (j + 1) - 1; //Middle
      }
    }
  }

  if (EdgeTangentCable) {
    for (int k = 0; k < 4; k++) {
      int ThisSide = m * (m + 8) + k * n * (m - n - 1) - 1;
      for (int j = 1; j <= o; j++) {
        for (int i = j; i < m - j; i++) {
          ThisSide++;
          if (i == j || i == m - j - 1) {
            Member++;
            End[Member][0] = m * k + i;
            End[Member][1] = ThisSide;
          } 
          else {
            Member++;
            End[Member][0] = ThisSide;
            End[Member][1] = ThisSide + (m - 1) - j * 2;
          }
        }
      }
    }
  }

  // Inner Cable Net
  for (int j = 0; j < m; j++) {
    for (int i = 0; i < m; i++) {
      if (i < m - 1) {
        Member++;
        End[Member][0] = 8 * m + j * m + i;
        End[Member][1] = 8 * m + j * m + i + 1;
      }
      if (j < m - 1) {
        Member++;
        End[Member][0] = 8 * m + j * m + i;
        End[Member][1] = 8 * m + j * m + i + m;
      }
    }
  }

  println("Check on last member number: Should be " + Member + " and is " + LastMember);
  LastMember = Member;

  for (Member = 0; Member <= LastMember; Member++) {
    if (Member < 16 * m) {
      L0[Member] = L(Member);
      EA[Member] = TrussEA;
      T0[Member] = TrussT0;
    } 
    else {
      if (UnstrainOnRelease) L0[Member] = 1;
      else L0[Member] = L(Member);
      EA[Member] = CableEA;
      T0[Member] = CableT0;
    }
  }
}

