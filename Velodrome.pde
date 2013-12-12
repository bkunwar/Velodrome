// ----------------------------------------------------------------------------------------------------
// ELLIPTICAL HYPERBOLIC PARABOLOID ANALYSIS PROGRAM
// ----------------------------------------------------------------------------------------------------
// Uses Alistair Day's Dynamic Relaxation technique, a simplified version of the Verlet algorithm
// Adapted from Chris William's SimpleDynamicRelaxation program
// by Bharat Kunwar
// ----------------------------------------------------------------------------------------------------

// import processing.opengl.*;

// prestressed final known
// udl final unknown
// unstrained final unknown


// Primary variables
int 
m = 10,             // 50
Variable        = 1, // 1 = CableT0 varies
ThisTest        = 0, 
WriteInterval   = 1000, 
DisplayInterval = 1000,
ReleaseTrigger  = 0, 
ClosingTrigger  = 4000;

// Member attributes in mm
float 
TrussRadius    = 40.0, 
TrussThickness = 10.0, 
CableRadius    = 10.0,
InnerTrussLength = 2500.0;
// Member attributes in N/mm2
float
TrussE = 200 * pow(10, 3), 
CableE = 200 * pow(10, 3), 
CableTensileStrength = 500, 
CableCompressiveStrength = 0, 
TrussTensileStrength     = 500, 
TrussCompressiveStrength = 500;
// Loads applied
float
dStrain = 0.0001, // 0.2% = 0.002
UDL      = 5.0,  // in N/mm
PLfactor = 10.0; // How much bigger than the UDL?

// Boolean settings
boolean 
FileOutputEnabled   = true, 
InitialGeometry     = true,
PLon                = false,
UDLon               = false,
SpecialCase         = true, 
SlidableCables      = false,
// Usually false
CableCompressible   = false,
// Usually true
Relaxable           = true,
TakeScreenshot      = true, 
ShowSupports        = true, 
// For program only
OutputComplete      = false, 
SummaryInitialized  = false, 
NewCycle            = false, 
EquilibriumReached  = false, 
FailureInitial      = false, 
FailureFinal        = false, 
Quit                = false, 
Released            = false;

// The following are required for operation
int
n, 
o, 
Iteration, 
CentralNode, 
LastNode, 
LastMember, 
LastCable, 
LastCableLink;
float
Ri, 
Ro,
Area, 
Circumference, 
Theta, 
ScaleFactor, 
TotalTrussLength, 
TotalCableLength, 
InitialRotation, 
OuterRingScale;
// Max or Min, Truss or Cable, Tension or Compression, Initial or Final
// Convert into an array one day, this is too repetative
float
MaxTrussTi, 
MaxTrussCi, 
MaxCableTi, 
MaxCableCi, 
MinTrussTi, 
MinTrussCi, 
MinCableTi, 
MinCableCi, 
MaxTrussTf, 
MaxTrussCf, 
MaxCableTf, 
MaxCableCf, 
MinTrussTf, 
MinTrussCf, 
MinCableTf, 
MinCableCf, 
dCableTi, 
dCableCi, 
dTrussTi, 
dTrussCi, 
dCableTf, 
dCableCf, 
dTrussTf, 
dTrussCf;

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

// The following are defined in the "setup" subroutine
float
TrussA, 
CableA, 
CableEA, 
TrussEA, 
CableTi, 
TrussTi, 
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
ColourContrast = 230, 
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
K;   // Node Stiffness

float [][]
Xi, // Initial coordinate
Xf, // Final coordinate
F, // Force
V; // Velocity

int [][] 
Fix;

// Member attribute matrices
float []
EA, // Young's Modulus times the Area
Ti, // Pretension
Tf, // Final Tension
Li, // Initial Length
Lf; // Final Length
int [][] 
End; // For each member, the nodes corresponding to the two ends of a given members

// Continuous cable properties
int [][][]
CL; // Members that make up a continuous cable in each direction

// ----------------------------------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------------------------------

void setup() {  
  // size(1200,750,OPENGL);
  // size(int(0.9*float(screen.width)),int(0.9*float(screen.height)),P3D);
  size(1200, 750, P3D);
  smooth();
  textFont(createFont(FontType, FontSize));
  textMode(SCREEN);

  println ("Initialising variables");  

  // m over 2 rounded down which gives the number of extra cables on each side
  n = int(m / 2);
  // m minus 1 over 2 rounded down which gives the number of extra members on each side
  o = int((m - 1) / 2);
  // Counting from zero
  LastNode = m * (m + 8) - 1;
  // Counting from zero
  LastMember = 2 * m * (m + 9) - 1;
  // Counting from zero, total number of cables in each direction
  LastCable = m - 1;  

  if (SpecialCase) {
    LastNode   += (4 * n) * (m - n - 1);
    LastMember += 4 * o * (m - o) + (4 * n) * (m - n - 1);
    LastCable  += 2 * o;
  }

  // Counting from zero, the maximum number of members per cable
  LastCableLink = LastCable + 1;

  // Rotate by Theta/2 and then 270 degrees 5*PI/4+PI/(4*m)
  InitialRotation = PI / 4 * (5 + 1 / float(m));
  // Renders the internal truss members the same length as the external members      
  OuterRingScale = PI * sqrt(3) / 4.0 / float(m) + 1;
  // Determine the pointer for the central node
  CentralNode = int(float(m) * (8 + float(m) / 2) + (float(m) + 1) % 2 * float(m) / 2);

  Xi = new float[LastNode + 1][3];
  Xf = new float[LastNode + 1][3];   
  F = new float[LastNode + 1][3];
  V = new float[LastNode + 1][3];
  K = new float[LastNode + 1];
  End = new int[LastMember + 1][2];  
  Fix = new int[LastNode + 1][3];  

  EA = new float[LastMember + 1];
  Ti = new float[LastMember + 1];
  Tf = new float[LastMember + 1];
  Li = new float[LastMember + 1];
  Lf = new float[LastMember + 1];

  CL = new int[LastCable + 1][LastCableLink + 1][2];

  // in mm2
  TrussA = PI*(pow(TrussRadius, 2)-pow(TrussRadius-TrussThickness, 2));
  CableA = PI*(pow(CableRadius, 2));

  // in N
  CableEA = CableE*CableA;
  TrussEA = TrussE*TrussA;
  
  // Increment of prestress in N, basically 10th of the cable strain capacity times its EA distributed among all its cables
  dP = dStrain * CableEA * m / LastCable;

  Ri = InnerTrussLength * 2 * float(m) / PI;
  Ro = Ri * OuterRingScale;
  
  ScaleFactor = float(height) / (3.0 * Ri);

  Area = PI * pow(Ri, 2) * xScale * yScale;
  Circumference = PI * Ri * (3 * (xScale + yScale) - sqrt((3 * xScale + yScale) * (xScale + 3 * yScale)));

  //In z, Plan = 0,  Elevation = 0,     Isometric = -PI/3;
  zRot = 0;
  //In x, Plan = 0,  Elevation = -PI/2, Isometric = -PI/3;
  xRot = 0;
  xTrans = 0.0;
  yTrans = 0.0;  

  Iteration = 0;
  Released = false;
  
  ConstructNodes();
  ConstructMembers();
  ConstructCableLinks();
  ResetNodeStiffness();
  CalculateTotalLength();
  ResetPointers();
  AssignMemberAttributes();
  InitiateSummaryOutput();
  InitiateOutput();
}

// ----------------------------------------------------------------------------------------------------
// CONSTRUCTORS
// ----------------------------------------------------------------------------------------------------

// Construct the nodes of the structure
void ConstructNodes() {
  int Node = -1;

  // Construct the nodes in the Internal Ring
  for (int i = 0; i < 4 * m; i++) {
    Node++;
    Theta = 2 * PI * i / (4 * m) + InitialRotation;
    Xi[Node][0] = xScale * Ri * cos(Theta); //Position in x axis
    Xi[Node][1] = yScale * Ri * sin(Theta); //Position in y axis
    if ((i + 1) % 1 == 0) for (int xyz = 0; xyz <= 2; xyz++) Fix[Node][xyz] = 1;
  }

  // Construct the nodes in the External Ring
  for (int i = 0; i < 4 * m; i++) {
    Node++;
    Theta = 2 * PI * i / (4 * m) + PI / (4 * m) + InitialRotation;
    Xi[Node][0] = xScale * Ro * cos(Theta); //Position in x axis
    Xi[Node][1] = yScale * Ro * sin(Theta); //Position in y axis
    if ((i + 1) % 1 == 0) for (int xyz = 0; xyz <= 2; xyz++) Fix[Node][xyz] = 1;
  }

  // Construct the nodes in the Central Cable Net
  for (int j = 0; j < m; j++) {
    for (int i = 0; i < m; i++) {
      Node++;
      Xi[Node][0] = Xi[i][0];     //Position in x axis
      Xi[Node][1] = Xi[m + j][1]; //Position in y axis
    }
  }

  // Construct the nodes in the Periphery Cable Net
  for (int k = 0; k < 4 && SpecialCase; k++) {
    for (int j = 1; j <= n; j++) {
      for (int i = j; i < m - j; i++) {
        Node++;
        int xNode = (i + m * k) * ((k + 1) % 2) + (j - 1 + m * k) * (k % 2);
        int yNode = (i + m * k) * (k % 2) + (j - 1 + m * k) * ((k + 1) % 2);
        Xi[Node][0] = Xi[xNode][0]; //Position in x axis
        Xi[Node][1] = Xi[yNode][1]; //Position in y axis
      }
    }
  }

  println("Check on last node number: Should be " + Node + " and is " + LastNode);
  LastNode = Node;

  // Determine the co-ordinate the nodes in z-axis
  for (Node = 0; Node <= LastNode; Node++) {
    Xi[Node][2] = zScale * (pow(Xi[Node][0]/pow(xScale, 1), 2) - pow(Xi[Node][1]/pow(yScale, 1), 2)) / Ri; //Position in z axis
  }

  // We want to start from the final geometry and work our way backwards so the final geometry is saved as the strained geometry
  for (Node = 0; Node <= LastNode; Node++) {
    arrayCopy(Xi[Node], Xf[Node]);
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
  if (SpecialCase) {
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
        if (i > 0 && i < m - 1 && SpecialCase) ThisSide += 0;
        else ThisSide = 0; //Edge
        End[Member][0] = ThisSide + i; //Edge            
        End[Member][1] = 8 * m + (m * j) + i; //Middle
      }
      if (j == (m - 1)) {
        //Bottom         
        if (i > 0 && i < m - 1 && SpecialCase) ThisSide += 2 * n * (m - n - 1); // Edge
        else ThisSide = 2 * m; // Corner
        Member++;
        End[Member][0] = ThisSide + m - 1 - i;
        End[Member][1] = 8 * m + (m * j) + i; // Net
      }
      if (i == 0) {
        //Left
        Member++;
        if (j > 0 && j < m - 1 && SpecialCase) ThisSide += 3 * n * (m - n - 1);
        else ThisSide = 3 * m; //Edge   
        End[Member][0] = ThisSide + m - 1 - j; //Edge                 
        End[Member][1] = 8 * m + (m * j); //Middle
      }
      if (i == (m - 1)) {
        //Right
        Member++;
        if (j > 0 && j < m - 1 && SpecialCase) ThisSide += n * (m - n - 1);
        else ThisSide = m;
        End[Member][0] = ThisSide + j; //Edge                               
        End[Member][1] = 8 * m + m * (j + 1) - 1; //Middle
      }
    }
  }


  for (int k = 0; k < 4 && SpecialCase; k++) {
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

  CalculateLi();
}

void ConstructCableLinks() {
  float MinX = 0;
  float MaxX = 0;

  for (int xy = 0; xy <= 1; xy++) {
    if (xy == 0) {
      MinX = Xf[int(7.5 * float(m)) - 1][0];
      MaxX = Xf[int(5.5 * float(m)) - 1][0];
    } 
    else if (xy == 1) {
      MinX = Xf[int(4.5 * float(m)) - 1][1];
      MaxX = Xf[int(6.5 * float(m)) - 1][1];
    }

    float ThisX = MinX;

    for (int Cable = 0; Cable <= LastCable; Cable++) {

      float ThatX = MaxX;

      // Find the first row
      for (int Node = 8 * m; Node <= LastNode; Node++) {
        if (Xf[Node][xy] > ThisX && Xf[Node][xy] < ThatX) ThatX = Xf[Node][xy];
      }

      ThisX = ThatX;

      int CableLink = 0;
      for (int Member = 16 * m; Member <= LastMember; Member++) {
        float ThisEndX = Xf[End[Member][0]][xy];
        float ThatEndX = Xf[End[Member][1]][xy];
        float Tolerance = Li[Member] * 0.001; // 0.1% of the length
        float UpperLimit = ThisX + Tolerance;
        float LowerLimit = ThisX - Tolerance;
        // println(Member + ": UpperLimit " + UpperLimit + " ThisEndX " + ThisEndX + " ThatEndX " + ThatEndX + " LowerLimit " + LowerLimit);        
        if (UpperLimit > ThisEndX && LowerLimit < ThisEndX && UpperLimit > ThatEndX && LowerLimit < ThatEndX) {
          // println(Member+": "+Cable+" "+CableLink+" "+xy+" | "+LastCableLink);
          CL[Cable][CableLink][xy] = Member;
          CableLink++;
        }
      }
    }
  }
}  

// Assign the attributes to each member
void AssignMemberAttributes() {
  
  // This is the initial prestress that we want in the cable
  if (Variable != 1 || !FileOutputEnabled) 
  CableTi = dP;
  // For the initial stress distribution, we know that the truss is unstressed
  TrussTi = 0;
  
  for (int Member = 0; Member <= LastMember; Member++) {
    if (Member < 16 * m) {
      // For the truss members, Tf and Li are unknown as we  do not know the unstrained geometry or the final stress distribution in the members
      // However, we do know Ti and Lf, as the initial stress distribution is defined and that the final geometry is the specified geometry

      // Li[Member] = ?
      // Tf[Member] = ? 

      Lf[Member] = Li[Member];
      Ti[Member] = TrussTi;

      EA[Member] = TrussEA;
    } 
    else {
      // For the cable members as well, Tf and Li are unknown as we  do not know the unstrained geometry or the final stress distribution in the members
      // However, we do know Ti and Lf, as the initial stress distribution is defined and that the final geometry is the specified geometry   

      // Li[Member] = ?
      // Tf[Member] = ? 

      Lf[Member] = Li[Member];
      Ti[Member] = CableTi;

      EA[Member] = CableEA;
    }
  }  
}
