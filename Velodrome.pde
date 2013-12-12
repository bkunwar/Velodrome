int m = 50,
    // Variable: 1 = m varies, 2 = CableT0 Varies
    Variable = 2,
    WriteInterval = 50,
    ReleaseTrigger = 200,
    SetLengthTrigger = 200, // Must be less than or equal to ReleaseTrigger
    ClosingTrigger = 20000;

float // in mm
      TrussRadius    = 200.0,
      TrussThickness = 50.0,
      CableRadius    = 5.0,

      // in mm
      InnerTrussLength = 2500.0,
      
      // in N/mm2
      TrussE = 200*pow(10,3),
      CableE = 200*pow(10,3),

      TrussA = PI*(pow(TrussRadius,2)-pow(TrussRadius-TrussThickness,2)),
      CableA = PI*(pow(CableRadius,2)),

      // in N
      CableEA = CableE*CableA,  
      TrussEA = TrussE*TrussA,

      P0 = 0.02*CableEA,

      // in N
      CableT0 = P0,
      TrussT0 = 0,              
      Load = CableT0,
      
      RelaxationRate = 0.98;

char Colour = 'C';// F=Force, M=Member, A=FirstMember, C=Compression

boolean Relaxable           = true, 
        ConstantCableLength = false,
        EdgeTangentCable    = true,
        CableCompressible   = false,
        OutputEnabled       = true,
        Fast                = false,
        Released            = false;      

void setup() {
  // size(1200,750,OPENGL);
  // size(int(0.9*float(screen.width)),int(0.9*float(screen.height)),P3D);
  size(1200, 750, P3D);
  smooth();
  textFont(createFont("Courier New", 20));
  frameRate(30);
  textMode(SCREEN);

  // m over 2 rounded down which gives the number of extra cables on each side
  n = int(m / 2);
  // m minus 1 over 2 rounded down
  o = int((m - 1) / 2);
  // Counting from zero
  LastNode = m * (m + 8) + (4 * n) * (m - n - 1) - 1;
  // Counting from zero
  LastMember = 2 * m * (m + 9) + 4 * o * (m - o) + (4 * n) * (m - n - 1) - 1;
  //Central Node
  // Rotate by Theta/2 and then 270 degrees 5*PI/4+PI/(4*m)
  InitialRotation = PI / 4 * (5 + 1 / float(m));
  // Renders the internal truss members the same length as the external members      
  OuterRingScale = PI * sqrt(3) / 4.0 / float(m) + 1;

  CentralNode = int(float(m) * (8 + float(m) / 2) + (float(m) + 1) % 2 * float(m) / 2);
  Coord = new float[LastNode + 1][3];
  Force = new float[LastNode + 1][3];
  Veloc = new float[LastNode + 1][3];
  End = new int[LastMember + 1][2];
  Fixed = new int[LastNode + 1][3];
  Stiffness = new float[LastNode + 1];
  EA = new float[LastMember + 1];
  T0 = new float[LastMember + 1];
  L0 = new float[LastMember + 1];
  TensionCoefficient = new float[LastMember + 1];

  zRot = 0.0;
  xRot = 0.0;
  xTrans = 0.0;
  yTrans = 0.0;

  xScale = 1.0;
  yScale = 0.6; //6
  zScale = 0.8; //0.15
  NetScale = 1 / sqrt(2) * (1 - PI / 4 / float(m)); // To make the net fit within the truss ring
  WidestInnerRadius = InnerTrussLength * 2 * float(m) / PI;
  ScaleFactor = float(height) / (3.0 * WidestInnerRadius);

  Area = PI * pow(WidestInnerRadius, 2) * xScale * yScale;
  Circumference = PI * WidestInnerRadius * (3 * (xScale + yScale) - sqrt((3 * xScale + yScale) * (xScale + 3 * yScale)));

  Iteration = 0;

  if (OutputEnabled) {
    int FileNumber;
    for (FileNumber = 0; FileExists(FileNumber); FileNumber++);
    Output = createWriter(dataPath(FileName + FileNumber + FileExtension));
    println("Output will be saved to the file: " + FileName + FileNumber + FileExtension);
  }

  {
    int Node = -1;

    // Internal Ring
    for (int i = 0; i < 4 * m; i++) {
      Node++;
      Theta = 2 * PI * i / (4 * m) + InitialRotation;
      Coord[Node][0] = xScale * WidestInnerRadius * cos(Theta); //Position in x axis
      Coord[Node][1] = yScale * WidestInnerRadius * sin(Theta); //Position in y axis
      if ((i + 1) % 1 == 0) for (int xyz = 0; xyz <= 2; xyz++) Fixed[Node][xyz] = 1;
    }

    // External Ring
    for (int i = 0; i < 4 * m; i++) {
      Node++;
      Theta = 2 * PI * i / (4 * m) + PI / (4 * m) + InitialRotation;
      Coord[Node][0] = xScale * OuterRingScale * WidestInnerRadius * cos(Theta); //Position in x axis
      Coord[Node][1] = yScale * OuterRingScale * WidestInnerRadius * sin(Theta); //Position in y axis
      if ((i + 1) % 1 == 0) for (int xyz = 0; xyz <= 2; xyz++) Fixed[Node][xyz] = 1;
    }

    // Cable Net
    for (int j = 0; j < m; j++) {
      for (int i = 0; i < m; i++) {
        Node++;
        Coord[Node][0] = Coord[i][0]; //Position in x axis
        Coord[Node][1] = Coord[m + j][1]; //Position in y axis
        /*
          Coord[Node][0]=xScale*NetScale*InnerRadius*(i-float(m-1))/Divider; //Position in x axis
          Coord[Node][1]=yScale*NetScale*InnerRadius*(j-float(m-1))/Divider; //Position in y axis
          */
      }
    }

    for (int k = 0; k < 4; k++) {
      for (int j = 1; j <= n; j++) {
        for (int i = j; i < m - j; i++) {
          Node++;
          int xNode = (i + m * k) * ((k + 1) % 2) + (j - 1 + m * k) * (k % 2);
          int yNode = (i + m * k) * (k % 2) + (j - 1 + m * k) * ((k + 1) % 2);
          Coord[Node][0] = Coord[xNode][0]; //Position in x axis
          Coord[Node][1] = Coord[yNode][1]; //Position in y axis          
        }
      }
    }

    println("Check on last node number: Should be " + Node + " and is " + LastNode);

    LastNode = Node;

    for (Node = 0; Node <= LastNode; Node++) {
      Coord[Node][2] = zScale * (pow(Coord[Node][0] / xScale, 2) - pow(Coord[Node][1] / yScale, 2)) / WidestInnerRadius; //Position in z axis
    }
  } {
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
            } else {
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
        L0[Member] = Length(Member);
        EA[Member] = TrussEA;
        T0[Member] = TrussT0;
      } else {
        if (ConstantCableLength) L0[Member] = 1;
        else L0[Member] = Length(Member);
        EA[Member] = CableEA;
        T0[Member] = CableT0;
      }
    }
  }

  ResetNodeStiffness(0, LastMember);
  CalculateMemberLength();

  X0 = Coord[int(5.5 * float(m)) - 1][0] - Coord[int(7.5 * float(m)) - 1][0];
  Y0 = Coord[int(6.5 * float(m)) - 1][1] - Coord[int(4.5 * float(m)) - 1][1];
  Z0 = Coord[CentralNode][2];
}
