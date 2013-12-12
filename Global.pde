//This program uses Alistair Day's Dynamic Relaxation technique, a simplified version of the Verlet algorithm
//import processing.opengl.*;

String FileName = "Output ";
String FileExtension = ".csv";
PrintWriter Output;

int Iteration,
    
    n,
    o,
    CentralNode,    
    LastNode,
    LastMember;

int[] ForceColour = new int[3];
      
int [][] End,
         Fixed;
          
float PTextX = 50.0,
      PTextY = 50.0,
      DTextY = 20.0,
      MyMouseX = 0.0,
      MyMouseY = 0.0,
      MyMouseXpressed = 0.0,
      MyMouseYpressed = 0.0,
      xTrans,
      yTrans,
      zRot,
      xRot,  
      MaxForce,
      ForceRatio,
      WidestInnerRadius,     
      dX,
      dY,
      dZ,
      X0,
      Y0,
      Z0,
      Theta,
      ScaleFactor,
      TotalTrussLength,
      TotalCableLength,
      InitialRotation,
      OuterRingScale;
      
float[] Stiffness,
        EA,
        T0,
        L0,
        Delta = new float[3];

float[][] Coord, 
          Force, 
          Veloc;

color Truss      = color(200,0,0,255),
      InnerRing  = color(0,200,0,255),
      OuterRing  = color(0,0,200,255),
      Cables     = color(255,255,255,255);

