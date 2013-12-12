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

      F = new float[LastNode + 1][3];

      HandleMousePress(); 
      HandleKeyPress();
      
      ResetNodeStiffness(0, LastMember);  

      for (int Member = 0; Member <= LastMember; Member++) {
        // Tension = T = T0 + (EA/L0)*(L - L0)
        // TC = T/L = EA/L0 + (T0 - EA)/L

/*
        T[Member] = T0[Member] + (EA[Member] / L0[Member]) * (L(Member) - L0[Member]);
        // Eliminate compressive forces from the cables if cables are not compressible
        if (!CableCompressible && Member >= 16 * m && T[Member] < 0) T[Member] = 0;
        float TC = T[Member] / L(Member);   
*/        
        
        T[Member] = T0[Member] + (EA[Member] / L(Member)) * (L0[Member]-L(Member));        
        
        // Eliminate compressive forces from the cables if cables are not compressible
        //if (!CableCompressible && Member >= 16 * m && T[Member] < 0) T[Member] = 0;

        float TC = T[Member] / L0[Member];

        // TC[Member] = EA[Member] / L0[Member] + (T0[Member] - EA[Member]) / L(Member);
        for (int xyz = 0; xyz <= 2; xyz++) {
          // FComponent = T/L * Delta[xyz]
          float Delta = X[End[Member][1]][xyz] - X[End[Member][0]][xyz];
          float ForceComponent = TC * Delta;
          F[End[Member][0]][xyz] -= ForceComponent;
          F[End[Member][1]][xyz] += ForceComponent;
        }
      }

      for (int Node = 0; Node <= LastNode; Node++) {
        for (int xyz = 0; xyz <= 2; xyz++) {
          if (Fix[Node][xyz] == 0) {
            float Force = F[Node][xyz] / K[Node];
            if (F[Node][xyz] == 0.0) Force = 0.0;
            V[Node][xyz] = Damping * V[Node][xyz] + Force;
            if (V[Node][xyz] > EquilibriumVelocity || V[Node][xyz] < -EquilibriumVelocity) EquilibriumReached = false;
            X[Node][xyz] += V[Node][xyz];
            //            if (Iteration%1000 == 0) println(V[Node][xyz]);
          }
        }
      }      

      // Keep count of iterations
      Iteration++;
    } 
    else {
      InnerCycle = DisplayInterval;
    }
  }

  NewCycle = false;
}

// Reset the stiffness of the node in the setup or once the member lengths have been reset
void ResetNodeStiffness(int First, int Last) {  
  for (int Member = First; Member <= Last; Member++) {
    float ThisLength = L(Member);//L0[Member];
    float EAoverL = EA[Member] / ThisLength;
    K[End[Member][0]] += EAoverL;
    K[End[Member][1]] += EAoverL;
  }
}

// Measure the displacements from the pointers
void MeasureDisplacements() {
  dX = X0 - (X[int(5.5 * float(m)) - 1][0] - X[int(7.5 * float(m)) - 1][0]);
  dY = Y0 - (X[int(6.5 * float(m)) - 1][1] - X[int(4.5 * float(m)) - 1][1]);
  dC = C0 - X[CentralNode][2];

  eX = dX / X0;
  eY = dY / Y0;
}

// Measure the maximum compressive and tensile stress in the cables and trusses
void MeasureMemberStress() {
  MaxTrussT = 0.0;  
  MaxTrussC = 0.0;
  MaxCableT = 0.0;
  MaxCableC = 0.0;

  Failure = false;

  for (int Member = 0; Member < 16 * m; Member++) {
    float Stress = T[Member] / TrussA;
    if (Stress > 0) {
      if (Stress > MaxTrussT) MaxTrussT = Stress;
      if (Stress < MinTrussT) MinTrussT = Stress;
    } else if (Stress < 0) {
      if (Stress < MaxTrussC) MaxTrussC = Stress;  
      if (Stress > MinTrussC) MinTrussC = Stress;
    }
    if (MaxTrussT > TrussST) Failure = true;
    if (MaxTrussC < -TrussSC) Failure = true;    
  }

  for (int Member = 16 * m; Member <= LastMember; Member++) {
    float Stress = T[Member] / CableA;
    if (Stress > 0) {
      if (Stress > MaxCableT) MaxCableT = Stress;
      if (Stress < MinCableT) MinCableT = Stress;
    } else if (Stress < 0) {
      if (Stress < MaxCableC) MaxCableC = Stress;
      if (Stress > MinCableC) MinCableC = Stress;
    }    
    if (MaxCableT > CableST) Failure = true;
    if (MaxCableC < -CableSC) Failure = true;        
  }

  dCableT = MaxCableT - MinCableT;
  dCableC = MinCableC - MaxCableC;
  
  dTrussT = MaxTrussT - MinTrussT;
  dTrussC = MinTrussC - MaxTrussC;
}

// Determine the length of the specified Member
float L(int Member) {
  float LengthSq = 0.0;
  for (int xyz = 0; xyz <= 2; xyz++)
    LengthSq += pow(X[End[Member][1]][xyz] - X[End[Member][0]][xyz], 2);
  return sqrt(LengthSq);
}

// Determine the total lenth of member from StartMember to EndMember
float TotalMemberLength(int StartMember, int EndMember) {
  float TotalLength = 0.0;
  for (int Member = StartMember; Member <= EndMember; Member++) TotalLength += L0[Member];
  return TotalLength;
}

//  Determine how much steel has been used in total for the cables ands trusses
void CalculateTotalLength() {
  TotalTrussLength = TotalMemberLength(0, 16 * m - 1);
  TotalCableLength = TotalMemberLength(16 * m, LastMember);
  println("Calculating Total Length");
}

// Reset the original length of the members to the deformed length so that they become unstrained
void ResetMemberLength(int First, int Last) {
  for (int Member = First; Member <= Last; Member++) L0[Member] = L(Member);
  ResetNodeStiffness(First, Last);
  CalculateTotalLength();
}

// Reset the pointers used to calculate the displacements
void ResetPointers() {
  X0 = X[int(5.5 * float(m)) - 1][0] - X[int(7.5 * float(m)) - 1][0];
  Y0 = X[int(6.5 * float(m)) - 1][1] - X[int(4.5 * float(m)) - 1][1];
  C0 = X[CentralNode][2];  
  println("Resetting Pointers");
}

// Release the perimeter nodes in the x and y axis
void Release() {
  for (int i = 0; i < 8 * m; i++)
    for (int xyz = 0; xyz <= 1; xyz++) Fix[i][xyz] = 0;
  Released = true;
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
    if (key == 'g') for (int Node = 8 * m; Node <= LastNode; Node++) F[Node][0] -= Load / (LastNode - 16 * m + 1);
    if (key == 'j') for (int Node = 8 * m; Node <= LastNode; Node++) F[Node][0] += Load / (LastNode - 16 * m + 1);
    if (key == 'y') for (int Node = 8 * m; Node <= LastNode; Node++) F[Node][1] -= Load / (LastNode - 16 * m + 1);
    if (key == 'h') for (int Node = 8 * m; Node <= LastNode; Node++) F[Node][1] += Load / (LastNode - 16 * m + 1);
    if (key == 'k') for (int Node = 8 * m; Node <= LastNode; Node++) F[Node][2] -= Load / (LastNode - 16 * m + 1);
    if (key == 'o') for (int Node = 8 * m; Node <= LastNode; Node++) F[Node][2] += Load / (LastNode - 16 * m + 1);

    // Zoom in and out
    if (key == 'z') ScaleFactor *= 1 + 0.1 / float(DisplayInterval);
    if (key == 'x') ScaleFactor /= 1 + 0.1 / float(DisplayInterval);

    // Poke in
    if (key == 'p' || key == 'l') {
      int r = 1;
      if (key == 'l') r = -1;
      if (m % 2 == 0) {
        F[CentralNode][2] += r * Load / 4;
        F[CentralNode - 1][2] += r * Load / 4;
        F[CentralNode - m][2] += r * Load / 4;
        F[CentralNode - m - 1][2] += r * Load / 4;
      } 
      else {
        F[CentralNode][2] += r * Load;
      }
    }

    if (key == 'r' && !Released) {
      if (UnstrainOnRelease) ResetMemberLength(16 * m, LastMember);
      Release();
    }
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
  if (!Failure && !Quit) {
    if (Variable == 1) {
      m += dm;
    } 
    else if (Variable == 2) {
      CableT0 += dP;
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
      "TrussT0," +
      "CableT0," +
      "MaxTrussT," +
      "MaxTrussC," +
      "MaxCableT," +
      "MaxCableC," +      
      "UnstrainOnRelease," +                    
      "EdgeTangentCable");
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
  if (FileOutputEnabled && !OutputComplete) {
    if (Iteration % WriteInterval == 0) {
      Output.println(Iteration + "," + 
        dX + "," + 
        dY + "," + 
        dC);
    }

    if (Iteration == ReleaseTrigger) {
      if (UnstrainOnRelease) ResetMemberLength(16 * m, LastMember);
      Release();
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
        TrussT0 + "," + 
        CableT0 + "," + 
        MaxTrussT + "," +
        MaxTrussC + "," +
        MaxCableT + "," +
        MaxCableC + "," +          
        UnstrainOnRelease + "," +                      
        EdgeTangentCable);
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
  Output.println(",,,,TrussT0," + TrussT0);
  Output.println(",,,,CableT0," + CableT0);
  Output.println(",,,,MaxTrussT," + MaxTrussT);
  Output.println(",,,,MaxTrussC," + MaxTrussC);    
  Output.println(",,,,MaxCableT," + MaxCableT);
  Output.println(",,,,MaxCableC," + MaxCableC);  
  Output.println(",,,,UnstrainOnRelease," + UnstrainOnRelease);
  Output.println(",,,,EdgeTangentCable," + EdgeTangentCable);
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
    if (Member < 16 * m) {
      float Stress = T[Member] / TrussA;
      if (Stress > 0) stroke(0, 255 - ColourContrast + int(float(ColourContrast) * Stress / dTrussT), TrussColour, 255);  
      else if (Stress < 0) stroke(255 - ColourContrast + int(float(ColourContrast) * abs(Stress) / dTrussC), 0, TrussColour, 255);
      else stroke(0, 0, 0, 255);
    } 
    else {
      float Stress = T[Member] / CableA;
      if (Stress > 0) stroke(0, 255 - ColourContrast + int(float(ColourContrast) * Stress / dCableT), CableColour, 255);  
      else if (Stress < 0) stroke(255 - ColourContrast + int(float(ColourContrast) * abs(Stress) / dCableC), 0, CableColour, 255);
      else stroke(0, 0, 0, 255);
    }   
    break;
  }
}

// Display text on the window
void DisplayText() {
  int TextCount = 0;
  if (FileOutputEnabled) {
    fill(255, 0, 0, 255);    
    if (Iteration < ClosingTrigger) {
      text("Writing to "+CurrentSummaryFileName+", "+CurrentFileName, PTextX, PTextY + DTextY * TextCount);
    } 
    else {
      text(CurrentSummaryFileName+ " complete.", PTextX, PTextY + DTextY * TextCount);
    }
    TextCount++;
  }  
  fill(0, 0, 0, 255);
  text("Iteration        " + Iteration, PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("Equilibrium      " + EquilibriumReached, PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("Failure          " + Failure, PTextX, PTextY + DTextY * TextCount);
  TextCount++;  
  text("xRot             " + xRot, PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("zRot             " + zRot, PTextX, PTextY + DTextY * TextCount);
  TextCount++;  
  text("m                " + m, PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("x Radius m       " + (WidestInnerRadius * xScale / pow(10, 3)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("y Radius m       " + (WidestInnerRadius * yScale / pow(10, 3)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("Circumference m  " + (Circumference / pow(10, 3)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("Ellipse Area m2  " + (Area / pow(10, 6)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("Sum Truss L Km:  " + (TotalTrussLength / pow(10, 6)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("Sum Cable L Km:  " + (TotalCableLength / pow(10, 6)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("Scale Factor     " + ScaleFactor, PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("Load kN          " + (Load / pow(10, 3)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("Cable T0 N/mm2   " + (CableT0 / CableA), PTextX, PTextY + DTextY * TextCount);
  TextCount++;  
  text("dY/dX            " + (dY / dX), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("dX mm            " + dX, PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("dY mm            " + dY, PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("dC mm            " + dC, PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  
  fill(0, 255 - ColourContrast + int(float(ColourContrast) * MaxTrussT / dTrussT), TrussColour, 255);
  text("MaxTrussT N/mm2  " + (MaxTrussT / pow(10, 0)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  fill(0, 255 - ColourContrast + int(float(ColourContrast) * MinTrussT / dTrussT), TrussColour, 255);
  text("MinTrussT N/mm2  " + (MinTrussT / pow(10, 0)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;  
  fill(255 - ColourContrast + int(float(ColourContrast) * abs(MaxTrussC) / dTrussC), 0, TrussColour, 255);
  text("MaxTrussC N/mm2  " + (MaxTrussC / pow(10, 0)), PTextX, PTextY + DTextY * TextCount); 
  TextCount++;
  fill(255 - ColourContrast + int(float(ColourContrast) * abs(MinTrussC) / dTrussC), 0, TrussColour, 255);
  text("MinTrussC N/mm2  " + (MinTrussC / pow(10, 0)), PTextX, PTextY + DTextY * TextCount); 
  TextCount++;
  
  fill(0, 255 - ColourContrast + int(float(ColourContrast) * MaxCableT / dCableS), CableColour, 255);
  text("MaxCableT N/mm2  " + (MaxCableT / pow(10, 0)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  fill(0, 255 - ColourContrast + int(float(ColourContrast) * MaxCableT / dCableS), CableColour, 255);
  text("MaxCableT N/mm2  " + (MaxCableT / pow(10, 0)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;  
  fill(255 - ColourContrast + int(float(ColourContrast) * abs(MaxCableC) / dCableC), 0, CableColour, 255);
  text("MaxCableC N/mm2  " + (MaxCableC / pow(10, 0)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;    
  fill(255 - ColourContrast + int(float(ColourContrast) * abs(MaxCableC) / dCableC), 0, CableColour, 255);
  text("MinCableC N/mm2  " + (MaxCableC / pow(10, 0)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;    
}

