void draw() {
  MeasureDisplacements();
  MeasureMemberStress();

  // Start Drawing
  background(255, 255, 255);  
  DisplayText();
  translate(float(width) / 2.0, float(height) / 2.0);
  ortho(-float(width) / 2.0, float(width) / 2.0, -float(height) / 2.0, float(height) / 2.0, -width, width);
  rotateX(-xRot);
  rotateZ(-zRot);
  translate(xTrans * cos(zRot) - yTrans * sin(zRot), yTrans * cos(zRot) + xTrans * sin(zRot), yTrans * sin(xRot));
  scale(ScaleFactor);
  smooth();
  strokeWeight(1.0); //P3D //strokeWeight(1.0/ScaleFactor);//OPENGL

  float X[][] = new float[LastNode + 1][3]; 

  if (InitialGeometry) X = Xi;
  else X = Xf;

  // Draw the members
  for (int Member = 0; Member <= LastMember; Member++) {
    // Colour the member 
    DetermineMemberColour(Member);
    // Draw the line to indicate the member
    line(X[End[Member][0]][0], X[End[Member][0]][1], X[End[Member][0]][2], X[End[Member][1]][0], X[End[Member][1]][1], X[End[Member][1]][2]);
  }

  // Draw the supports if enabled
  for (int Node = 0; Node <= LastNode && ShowSupports; Node++) {
    // Draw the line to indicate the member
    stroke(255, 0, 0, 100); 
    if (Fix[Node][2] == 1) {
      int SupportDetail = 24;
      for (int i = 0; i < SupportDetail; i++) {
        float Phi = i * 2 * PI / SupportDetail;
        float SupportHeight = WidestInnerRadius / 50;
        float SupportRadius = WidestInnerRadius / 100;
        line(X[Node][0], X[Node][1], X[Node][2], X[Node][0] + SupportRadius * cos(Phi), X[Node][1] + SupportRadius * sin(Phi), X[Node][2] - SupportHeight);
      }
    }
  }  
  // Finish Drawing 

  Relax();
}

// ----------------------------------------------------------------------------------------------------
// ANALYSIS AND SECONDARY PROCEDURES
// ----------------------------------------------------------------------------------------------------

// Relaxation algorithm
void Relax() {
  EquilibriumReached = true;  

  for (int InnerCycle = 0; InnerCycle < DisplayInterval && Relaxable; InnerCycle++) {   

    HandleFileOutput();

    if (!NewCycle&&!OutputComplete) {

      /*
        T[Member] = Tf[Member] + (EA[Member] / L0[Member]) * (L(Member) - L0[Member]);
       // Eliminate compressive forces from the cables if cables are not compressible
       
       if (!CableCompressible && Member >= 16 * m && T[Member] < 0) T[Member] = 0;
       
       float TC = T[Member] / L(Member);
       TC[Member] = EA[Member] / L0[Member] + (Tf[Member] - EA[Member]) / L(Member);            
       
       Ti[Member] = Tf[Member] - EA[Member] * (Lf[Member] - Li[Member]) / Li[Member];
       TC = Ti[Member] / Li[Member];
       for (int xyz = 0; xyz <= 2; xyz++) {
       // FComponent = T/L * Delta[xyz]
       float Delta = Xi[End[Member][1]][xyz] - Xi[End[Member][0]][xyz];
       float ForceComponent = TC * Delta;
       F[End[Member][0]][xyz] += ForceComponent;
       F[End[Member][1]][xyz] -= ForceComponent;
       }          
       //}    
       */

      // Tension = T = T0 + (EA/L0)*(L - L0)
      // TC = T/L = EA/L0 + (T0 - EA)/L

      F = new float[LastNode + 1][3];

      HandleMousePress(); 
      HandleKeyPress();
      
      ApplyCableLoads();      
      ResetNodeStiffness();      

      for (int Member = 0; Member <= LastMember; Member++) {

        float TC;

        Tf[Member] = Ti[Member] + (EA[Member] / Li[Member]) * (Lf[Member] - Li[Member]);
        if (!CableCompressible && Member >= 16 * m && Tf[Member] < 0) Tf[Member] = 0;

        //if (Iteration%1000 == 0) println(Tf[Member]);

        TC = -Tf[Member] / Li[Member];

        for (int xyz = 0; xyz <= 2; xyz++) {
          // FComponent = T/L * Delta[xyz]
          float Delta = Xi[End[Member][1]][xyz] - Xi[End[Member][0]][xyz];
          float ForceComponent = TC * Delta;
          F[End[Member][0]][xyz] += ForceComponent;
          F[End[Member][1]][xyz] -= ForceComponent;
        }
      }

      for (int Node = 0; Node <= LastNode; Node++) {
        for (int xyz = 0; xyz <= 2; xyz++) {
          if (Fix[Node][xyz] == 0) {
            float Force = F[Node][xyz] / K[Node];
            if (F[Node][xyz] == 0.0) Force = 0.0;
            V[Node][xyz] = Damping * V[Node][xyz] + Force;
            // If the rate of relaxation has drastically slowed down, assume static equilibrium
            if (V[Node][xyz] > EquilibriumVelocity || V[Node][xyz] < -EquilibriumVelocity) EquilibriumReached = false;
            Xi[Node][xyz] += V[Node][xyz];
            //            if (Iteration%1000 == 0) println(V[Node][xyz]);
          }
        }
      }

      CalculateLi();
      SlideCables();      

      // Keep count of iterations
      Iteration++;
    } 
    else {
      InnerCycle = DisplayInterval;
    }
  }

  NewCycle = false;
}

// Slide the cables slightly to even up the stress along a continuous cable
void SlideCables() {
  if (SlidableCables) {  
    for (int xy = 0; xy <= 1; xy++) {
      for (int Cable = 0; Cable <= LastCable; Cable++) {
        float SumLi = 0.0;
        float SumLf = 0.0;
        for (int CableLink = 0; CableLink <= LastCableLink; CableLink++) {
          int Member = CL[Cable][CableLink][xy];
          if (Member != 0) {
            SumLi += Li[Member];
            SumLf += Lf[Member];
          }
        }

        float TensionOverEA = (SumLf - SumLi) / SumLi;
        //if (Iteration%1000 == 0) println(TensionOverEA);
        for (int CableLink = 0; CableLink <= LastCableLink; CableLink++) {
          int Member = CL[Cable][CableLink][xy];
          if (Member != 0) {
            Lf[Member] = Li[Member] * (1 + TensionOverEA);
          }
        }
      }
    }
  }
}

// Reset the stiffness of the node in the setup or once the member lengths have been reset
void ResetNodeStiffness() {  
  for (int Member = 0; Member <= LastMember; Member++) {
    float EAoverL = EA[Member] / Li[Member];
    K[End[Member][0]] += EAoverL;
    K[End[Member][1]] += EAoverL;
  }
}

// Measure the displacements from the pointers
void MeasureDisplacements() {
  dX = X0 - (Xi[int(5.5 * float(m)) - 1][0] - Xi[int(7.5 * float(m)) - 1][0]);
  dY = Y0 - (Xi[int(6.5 * float(m)) - 1][1] - Xi[int(4.5 * float(m)) - 1][1]);
  dC = C0 - Xi[CentralNode][2];

  eX = dX / X0;
  eY = dY / Y0;
}

// Measure the maximum compressive and tensile stress in the cables and trusses
void MeasureMemberStress() {
  MinTrussTi = MaxTrussTi;
  MinTrussCi = MaxTrussCi;
  MaxTrussTi = 0.0;  
  MaxTrussCi = 0.0;

  MinTrussTf = MaxTrussTf;
  MinTrussCf = MaxTrussCf;
  MaxTrussTf = 0.0;  
  MaxTrussCf = 0.0;

  MinCableTi = MaxCableTi;
  MinCableCi = MaxCableCi;
  MaxCableTi = 0.0;
  MaxCableCi = 0.0;

  MinCableTf = MaxCableTf;
  MinCableCf = MaxCableCf;
  MaxCableTf = 0.0;
  MaxCableCf = 0.0;

  FailureInitial = false;
  FailureFinal   = false;  

  float Stress = 0.0;

  for (int Member = 0; Member < 16 * m; Member++) {
    // Initial stress state
    Stress = Ti[Member] / TrussA;
    if (Stress > 0) {
      if (Stress > MaxTrussTi) MaxTrussTi = Stress;
      if (Stress < MinTrussTi) MinTrussTi = Stress;
    } 
    else if (Stress < 0) {
      if (Stress < MaxTrussCi) MaxTrussCi = Stress;  
      if (Stress > MinTrussCi) MinTrussCi = Stress;
    }
    if (MaxTrussTi > TrussTensileStrength) FailureInitial = true;
    if (MaxTrussCi < -TrussCompressiveStrength) FailureInitial = true;

    // Final stress state
    Stress = Tf[Member] / TrussA;
    if (Stress > 0) {
      if (Stress > MaxTrussTf) MaxTrussTf = Stress;
      if (Stress < MinTrussTf) MinTrussTf = Stress;
    } 
    else if (Stress < 0) {
      if (Stress < MaxTrussCf) MaxTrussCf = Stress;  
      if (Stress > MinTrussCf) MinTrussCf = Stress;
    }
    if (MaxTrussTf > TrussTensileStrength) FailureFinal = true;
    if (MaxTrussCf < -TrussCompressiveStrength) FailureFinal = true;
  }

  for (int Member = 16 * m; Member <= LastMember; Member++) {
    // Initial stress state
    Stress = Ti[Member] / CableA;
    if (Stress > 0) {
      if (Stress > MaxCableTi) MaxCableTi = Stress;
      if (Stress < MinCableTi) MinCableTi = Stress;
    } 
    else if (Stress < 0) {
      if (Stress < MaxCableCi) MaxCableCi = Stress;
      if (Stress > MinCableCi) MinCableCi = Stress;
    }    
    if (MaxCableTi > CableTensileStrength) FailureInitial = true;
    if (MaxCableCi < -CableCompressiveStrength) FailureInitial = true; 

    // Final stress state
    Stress = Tf[Member] / CableA;
    if (Stress > 0) {
      if (Stress > MaxCableTf) MaxCableTf = Stress;
      if (Stress < MinCableTf) MinCableTf = Stress;
    } 
    else if (Stress < 0) {
      if (Stress < MaxCableCf) MaxCableCf = Stress;
      if (Stress > MinCableCf) MinCableCf = Stress;
    }    
    if (MaxCableTf > CableTensileStrength) FailureFinal = true;
    if (MaxCableCi < -CableCompressiveStrength) FailureFinal = true;
  }

  dCableTi = MaxCableTi - MinCableTi;
  dCableCi = MinCableCi - MaxCableCi;

  dTrussTi = MaxTrussTi - MinTrussTi;
  dTrussCi = MinTrussCi - MaxTrussCi;

  dCableTf = MaxCableTf - MinCableTf;
  dCableCf = MinCableCf - MaxCableCf;

  dTrussTf = MaxTrussTf - MinTrussTf;
  dTrussCf = MinTrussCf - MaxTrussCf;
}

// Reset the initial length of the members from the Xi co-ordinate matrix
void CalculateLi() {
  for (int Member = 0; Member <= LastMember; Member++) {
    float LengthSq = 0.0;
    for (int xyz = 0; xyz <= 2; xyz++)
      LengthSq += pow(Xi[End[Member][1]][xyz] - Xi[End[Member][0]][xyz], 2);
    Li[Member] = sqrt(LengthSq);
  }
}

// Determine the total lenth of member from StartMember to EndMember
float TotalMemberLength(int StartMember, int EndMember) {
  float TotalLength = 0.0;
  for (int Member = StartMember; Member <= EndMember; Member++) TotalLength += Li[Member];
  return TotalLength;
}

//  Determine how much steel has been used in total for the cables ands trusses
void CalculateTotalLength() {
  TotalTrussLength = TotalMemberLength(0, 16 * m - 1);
  TotalCableLength = TotalMemberLength(16 * m, LastMember);
  println("Calculating Total Length");
}

// Reset the pointers used to calculate the displacements
void ResetPointers() {
  X0 = Xi[int(5.5 * float(m)) - 1][0] - Xi[int(7.5 * float(m)) - 1][0];
  Y0 = Xi[int(6.5 * float(m)) - 1][1] - Xi[int(4.5 * float(m)) - 1][1];
  C0 = Xi[CentralNode][2];  
  println("Resetting Pointers");
}

// Release the perimeter nodes in the x and y axis
void Release() {
  for (int i = 0; i < 8 * m; i++)
    for (int xyz = 0; xyz <= 1; xyz++) Fix[i][xyz] = 0;
  Released = true;
}

void ApplyCableLoads () {
  for (int xyz = 0; xyz <=2; xyz++) {
    if (UDLon[xyz]) {
      for (int Node = 8 * m; Node <= LastNode; Node++)
        F[Node][xyz] += Load / (LastNode - 16 * m + 1);
    }
    if (PLon) {
      if (m % 2 == 0) {
        F[CentralNode][2] += Load / 4;
        F[CentralNode - 1][2] += Load / 4;
        F[CentralNode - m][2] += Load / 4;
        F[CentralNode - m - 1][2] += Load / 4;
      } 
      else {
        F[CentralNode][2] += Load;
      }
    }
  }
}

// ----------------------------------------------------------------------------------------------------
// INPUTS
// ----------------------------------------------------------------------------------------------------

// Handle the key presses
void HandleKeyPress() {
  if (keyPressed) {    
    // Waits till the current test finishes and closes the file and the summary file
    if (key == 'q') Quit = true;

    // Increase or decrease poke F
    if (key == 'a') Load *= 1 + 0.1 / float(DisplayInterval);
    if (key == 's') Load /= 1 + 0.1 / float(DisplayInterval);

    // UDL in x, y or z direction 
    if (key == 'o') for (int Node = 8 * m; Node <= LastNode; Node++) UDLon[0] = !UDLon[0];
    if (key == 'i') for (int Node = 8 * m; Node <= LastNode; Node++) UDLon[1] = !UDLon[1];
    if (key == 'l') for (int Node = 8 * m; Node <= LastNode; Node++) UDLon[2] = !UDLon[2];

    // Poke in
    if (key == 'p') PLon = !PLon;

    // Zoom in and out
    if (key == 'z') ScaleFactor *= 1 + 0.1 / float(DisplayInterval);
    if (key == 'x') ScaleFactor /= 1 + 0.1 / float(DisplayInterval);

    if (key == 'r' && !Released) Release();

    if (key == ',') InitialGeometry = true;
    if (key == '.') InitialGeometry = false;
  }
}

// Handle the mouse presses
void HandleMousePress() {
  if (mousePressed) {
    MyMouseXpressed = mouseX;
    MyMouseYpressed = mouseY;
    if (mouseButton == LEFT) {
      zRot += (MyMouseXpressed - MyMouseX) / 300.0;
      xRot += (MyMouseYpressed - MyMouseY) / 300.0;
    } 
    else {
      xTrans += (MyMouseXpressed - MyMouseX);
      yTrans += (MyMouseYpressed - MyMouseY);
    }
    MyMouseX = MyMouseXpressed;
    MyMouseY = MyMouseYpressed;
  } 
  else {
    MyMouseX = mouseX;
    MyMouseY = mouseY;
  }
}


// ----------------------------------------------------------------------------------------------------
// OUTPUTS
// ----------------------------------------------------------------------------------------------------

// Handle the change in variable after the end of every test
void HandleVariables() {
  ThisTest++;
  if (!FailureInitial && !FailureFinal && !Quit) {
    if (Variable == 1) {
      m += dm;
    } 
    else if (Variable == 2) {
      CableTi += dP;
    } 
    else if (Variable == 3) {
      TrussThickness -= dTT;
    }
    setup();
    NewCycle = true;
  } 
  else CloseSummaryOutput();
}

// Check if the specified file exists with the global extension
boolean FileExists(String TemporaryFile) {
  File f = new File(dataPath(TemporaryFile + FileExtension));
  if (f.exists()) return true;
  else return false;
}

// Setup the summary output
void InitiateSummaryOutput() {
  if (FileOutputEnabled && !SummaryInitialized) {
    int FileNumber;
    boolean FileFree = false;
    for (FileNumber = 0; !FileFree; FileNumber++) {
      CurrentSummaryFileName = SummaryFileName + FileNumber;
      CurrentSummaryFileName += "/" + CurrentSummaryFileName;      
      if (!FileExists(CurrentSummaryFileName)) FileFree = true;
    }
    SummaryOutput = createWriter(dataPath(CurrentSummaryFileName + FileExtension));
    SummaryOutput.println("Iteration," +
      "dX," +
      "dY," +
      "dC," +
      "X0," +
      "Y0," +
      "ZC," +
      "m," +
      "Variable," +
      "TrussRadius," +
      "TrussThickness," +
      "InnerTrussLength," +
      "TrussE," +
      "CableE," +
      "TrussA," +
      "CableA," +
      "TrussEA," +
      "CableEA," +
      "TrussTi," +
      "CableTi," +
      "MaxTrussTi," +
      "MaxTrussCi," +
      "MaxCableTi," +
      "MaxCableCi," +      
      "MinTrussTi," +
      "MinTrussCi," +
      "MinCableTi," +
      "MinCableCi," +
      "MaxTrussTf," +
      "MaxTrussCf," +
      "MaxCableTf," +
      "MaxCableCf," +      
      "MinTrussTf," +
      "MinTrussCf," +
      "MinCableTf," +
      "MinCableCf," +
      "SlidableCables," +
      "SpecialCase");
    SummaryInitialized = true;
  }
}

// Setup the output
void InitiateOutput() {
  if (FileOutputEnabled) {
    int FileNumber;
    boolean FileFree = false;
    for (FileNumber = 0; !FileFree; FileNumber++) {
      CurrentFileName = CurrentSummaryFileName + FileName + FileNumber;
      if (!FileExists(CurrentFileName)) FileFree = true;
    }
    Output = createWriter(dataPath(CurrentFileName+FileExtension));
    Output.println("Iteration," +
      "dX," +
      "dY," +
      "dC");
  }
}

// Handle the file output for each iteration
void HandleFileOutput() {
  if (Iteration == ReleaseTrigger) {
    Release();
  }  

  if (FileOutputEnabled && !OutputComplete) {
    if (Iteration % WriteInterval == 0) {
      Output.println(Iteration + "," + 
        dX + "," + 
        dY + "," + 
        dC);
    }

    if (Iteration == ClosingTrigger) {
      CloseOutput();
      SummaryOutput.println(
      Iteration + "," + 
        dX + "," + 
        dY + "," + 
        dC + "," + 
        X0 + "," + 
        Y0 + "," + 
        C0 + "," + 
        m + "," + 
        Variable + "," + 
        TrussRadius + "," + 
        TrussThickness + "," + 
        InnerTrussLength + "," + 
        TrussE + "," + 
        CableE + "," + 
        TrussA + "," + 
        CableA + "," + 
        TrussEA + "," + 
        CableEA + "," + 
        TrussTi + "," + 
        CableTi + "," + 
        MaxTrussTi + "," +
        MaxTrussCi + "," +
        MinTrussTi + "," +
        MinTrussCi + "," +
        MaxCableTi + "," +
        MaxCableCi + "," +          
        MinCableTi + "," +
        MinCableCi + "," +     
        MaxTrussTf + "," +
        MaxTrussCf + "," +
        MinTrussTf + "," +
        MinTrussCf + "," +
        MaxCableTf + "," +
        MaxCableCf + "," +          
        MinCableTf + "," +
        MinCableCf + "," +
        SlidableCables + "," +                  
        SpecialCase);
      if (TakeScreenshot) saveFrame(DataFolder + CurrentFileName + ".png");                            
      HandleVariables();
    }
  }
}

// Close the output
void CloseOutput() {
  Output.println(",,,,m," + m);
  Output.println(",,,,Variable," + Variable);
  Output.println(",,,,TrussRadius," + TrussRadius);
  Output.println(",,,,TrussThickness," + TrussThickness);
  Output.println(",,,,InnerTrussLength," + InnerTrussLength);
  Output.println(",,,,TrussE," + TrussE);
  Output.println(",,,,CableE," + CableE);
  Output.println(",,,,TrussA," + TrussA);
  Output.println(",,,,CableA," + CableA);
  Output.println(",,,,TrussEA," + TrussEA);
  Output.println(",,,,CableEA," + CableEA);
  Output.println(",,,,TrussTi," + TrussTi);
  Output.println(",,,,CableTi," + CableTi);
  Output.println(",,,,MaxTrussTi," + MaxTrussTi);
  Output.println(",,,,MaxTrussCi," + MaxTrussCi);    
  Output.println(",,,,MinTrussTi," + MinTrussTi);
  Output.println(",,,,MinTrussCi," + MinTrussCi);    
  Output.println(",,,,MaxCableTi," + MaxCableTi);
  Output.println(",,,,MaxCableCi," + MaxCableCi); 
  Output.println(",,,,MinCableTi," + MinCableTi);
  Output.println(",,,,MinCableCi," + MinCableCi);    
  Output.println(",,,,MaxTrussTf," + MaxTrussTi);
  Output.println(",,,,MaxTrussCf," + MaxTrussCi);    
  Output.println(",,,,MinTrussTf," + MinTrussTi);
  Output.println(",,,,MinTrussCf," + MinTrussCi);    
  Output.println(",,,,MaxCableTf," + MaxCableTi);
  Output.println(",,,,MaxCableCf," + MaxCableCi); 
  Output.println(",,,,MinCableTf," + MinCableTi);
  Output.println(",,,,MinCableCf," + MinCableCi);
  Output.println(",,,,SlidableCables," + SlidableCables);    
  Output.println(",,,,SpecialCase," + SpecialCase);
  Output.close();
  println("Finished writing to " + CurrentFileName);
}  

// Close the summary output
void CloseSummaryOutput() {
  SummaryOutput.close();
  println("Finished writing to " + CurrentSummaryFileName);
  OutputComplete = true;
}

// Determine the colour of the members
void DetermineMemberColour(int Member) {
  switch (Colour) {
  case 'A':
    if (Member == 0 || Member == 16 * m) stroke(200, 0, 0, 255);
    else stroke(0, 0, 0, 255);
    break;
  case 'F':
    float Stress = 0.0;
    float MinStressC = 0.0;    
    float MinStressT = 0.0;    
    float dStressC = 0.0;
    float dStressT = 0.0;
    int MemberColour;
    if (Member < 16 * m) {
      MemberColour = TrussColour;
      if (InitialGeometry) {
        Stress = Ti[Member] / TrussA;
        MinStressT = MinTrussTi;
        MinStressC = MinTrussCi;        
        dStressT = dTrussTi;
        dStressC = dTrussCi;
      } 
      else {
        Stress = Tf[Member] / TrussA;
        MinStressT = MinTrussTf;
        MinStressC = MinTrussCf;        
        dStressT = dTrussTf;
        dStressC = dTrussCf;
      }
    } 
    else {
      MemberColour = CableColour;
      if (InitialGeometry) {
        Stress = Ti[Member] / CableA;
        MinStressT = MinCableTi;        
        MinStressC = MinCableCi;            
        dStressT = dCableTi;
        dStressC = dCableCi;
      } 
      else {
        Stress = Tf[Member] / CableA;
        MinStressT = MinCableTf;                
        MinStressC = MinCableCf;                        
        dStressT = dCableTf;
        dStressC = dCableCf;
      }
    }

    float ColourFactor = 1;
    // The member is a tensile member
    if (Stress > 0) {
      if (dStressT != 0) ColourFactor = (Stress - MinStressT) / dStressT;
      stroke(0, 255 - ColourContrast + int(float(ColourContrast) * ColourFactor), MemberColour, 255);
    }
    // The member is a compression member
    else if (Stress < 0) {
      if (dStressC != 0) ColourFactor = abs(Stress - MinStressC) / dStressC;
      stroke(255 - ColourContrast + int(float(ColourContrast) * ColourFactor), 0, MemberColour, 255);
    }
    // The member is unstressed
    else stroke(0, 0, 0, 255);
    break;
  }
}

// Display text on the window
void DisplayText() {
  int NewLine = 0;

  text("[ CONTROLS ]    [ <mouse : Rotate ][ >mouse : Rotate ]    [ z: Zoom+ ][ x: Zoom- ]    [ r: Release ]  [ q: Stop Relaxation ]", PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("[ a: Load+ ][ s: Load- ][ o: UDL x ][ i: UDL y  ][ o: UDL z ][ p: Point Load z ][ <: Initial Geometry ][ >: Final Geometry ]", PTextX, PTextY + DTextY * NewLine);  
  NewLine++;  
  text("----------------------------------------------------------------------------------------------------------------------------", PTextX, PTextY + DTextY * NewLine);  
  NewLine++;  

  if (FileOutputEnabled) {
    fill(255, 0, 0, 255);    
    if (Iteration < ClosingTrigger) {
      text("Writing to "+CurrentSummaryFileName+", "+CurrentFileName, PTextX, PTextY + DTextY * NewLine);
    } 
    else {
      text(CurrentSummaryFileName+ " complete.", PTextX, PTextY + DTextY * NewLine);
    }
    NewLine++;
  } 

  fill(0, 0, 0, 255);
  text("Iteration        " + Iteration, PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("Equilibrium      " + EquilibriumReached, PTextX, PTextY + DTextY * NewLine);
  NewLine++; 
  text("Released         " + Released, PTextX, PTextY + DTextY * NewLine);
  NewLine++;   
  text("SlidableCables   " + SlidableCables, PTextX, PTextY + DTextY * NewLine);
  NewLine++;     
  text("SpecialCase      " + SpecialCase, PTextX, PTextY + DTextY * NewLine);
  NewLine++;  
  text("xRot             " + xRot, PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("zRot             " + zRot, PTextX, PTextY + DTextY * NewLine);
  NewLine++;  
  text("m                " + m, PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("x Radius m       " + (WidestInnerRadius * xScale / pow(10, 3)), PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("y Radius m       " + (WidestInnerRadius * yScale / pow(10, 3)), PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("Circumference m  " + (Circumference / pow(10, 3)), PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("Ellipse Area m2  " + (Area / pow(10, 6)), PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("Sum Truss L Km:  " + (TotalTrussLength / pow(10, 6)), PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("Sum Cable L Km:  " + (TotalCableLength / pow(10, 6)), PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("Scale Factor     " + ScaleFactor, PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("Load kN          " + (Load / pow(10, 3)), PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("Cable Ti N/mm2   " + (CableTi / CableA), PTextX, PTextY + DTextY * NewLine);
  NewLine++;  
  text("Cable Ti kN/DOF  " + (LastCable * CableTi), PTextX, PTextY + DTextY * NewLine);
  NewLine++;    
  text("Truss Ti N/mm2   " + (TrussTi / TrussA), PTextX, PTextY + DTextY * NewLine);
  NewLine++;  
  text("dY/dX            " + (dY / dX), PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("dX mm            " + dX, PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("dY mm            " + dY, PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("dC mm            " + dC, PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  NewLine++;

  boolean Failure;
  float MinTrussT;
  float MinTrussC;
  float MaxTrussT;
  float MaxTrussC;
  float MinCableT;
  float MinCableC;
  float MaxCableT;
  float MaxCableC;
  String Geometry;

  if (InitialGeometry) {
    Geometry = "Initial Geometry";
    Failure = FailureInitial;
    MinTrussT = MinTrussTi;
    MinTrussC = MinTrussCi;
    MaxTrussT = MaxTrussTi;  
    MaxTrussC = MaxTrussCi;
    MinCableT = MinCableTi;
    MinCableC = MinCableCi;
    MaxCableT = MaxCableTi;  
    MaxCableC = MaxCableCi;
  } 
  else {
    Geometry = "Final Geometry";    
    Failure = FailureFinal;
    MinTrussT = MinTrussTf;
    MinTrussC = MinTrussCf;
    MaxTrussT = MaxTrussTf;  
    MaxTrussC = MaxTrussCf;
    MinCableT = MinCableTf;
    MinCableC = MinCableCf;
    MaxCableT = MaxCableTf;  
    MaxCableC = MaxCableCf;
  } 

  text(Geometry, PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  text("Failure          " + Failure, PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  fill(0, 255, TrussColour, 255);
  text("MaxTrussT N/mm2  " + (MaxTrussT / pow(10, 0)), PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  fill(0, 255 - ColourContrast, TrussColour, 255);
  text("MinTrussT N/mm2  " + (MinTrussT / pow(10, 0)), PTextX, PTextY + DTextY * NewLine);
  NewLine++;  
  fill(255, 0, TrussColour, 255);
  text("MaxTrussC N/mm2  " + (MaxTrussC / pow(10, 0)), PTextX, PTextY + DTextY * NewLine); 
  NewLine++;
  fill(255 - ColourContrast, 0, TrussColour, 255);
  text("MinTrussC N/mm2  " + (MinTrussC / pow(10, 0)), PTextX, PTextY + DTextY * NewLine); 
  NewLine++;  
  fill(0, 255, CableColour, 255);
  text("MaxCableT N/mm2  " + (MaxCableT / pow(10, 0)), PTextX, PTextY + DTextY * NewLine);
  NewLine++;
  fill(0, 255 - ColourContrast, CableColour, 255);
  text("MinCableT N/mm2  " + (MinCableT / pow(10, 0)), PTextX, PTextY + DTextY * NewLine);
  NewLine++;  
  fill(255, 0, CableColour, 255);
  text("MaxCableC N/mm2  " + (MaxCableC / pow(10, 0)), PTextX, PTextY + DTextY * NewLine);
  NewLine++;    
  fill(255 - ColourContrast, 0, CableColour, 255);
  text("MinCableC N/mm2  " + (MinCableC / pow(10, 0)), PTextX, PTextY + DTextY * NewLine);
  NewLine++;
}

