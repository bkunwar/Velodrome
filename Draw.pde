void draw() {
  int TextCount = 0;
  background(0, 0, 0);
  if (mousePressed) {
    MyMouseXpressed = mouseX;
    MyMouseYpressed = mouseY;
    if (mouseButton == LEFT) {
      zRot += (MyMouseXpressed - MyMouseX) / 300.0;
      xRot += (MyMouseYpressed - MyMouseY) / 300.0;
    } else {
      xTrans += (MyMouseXpressed - MyMouseX);
      yTrans += (MyMouseYpressed - MyMouseY);
    }
    MyMouseX = MyMouseXpressed;
    MyMouseY = MyMouseYpressed;
  } else {
    MyMouseX = mouseX;
    MyMouseY = mouseY;
  }

  for (int Node = 0; Node <= LastNode; Node++)
  for (int xyz = 0; xyz <= 2; xyz++) Force[Node][xyz] = 0.0;

  if (keyPressed) {
    // Increase or decrease frame rate
    if (key == 'f') frameRate(30);
    if (key == 'd') frameRate(5);

    // Increase or decrease poke force
    if (key == 'a') Load *= 1.1;
    if (key == 's') Load /= 1.1;

    // UDL in x, y or z direction 
    if (key == 'g') for (int Node = 8 * m; Node <= LastNode; Node++) Force[Node][0] -= Load / (LastNode - 16 * m + 1);
    if (key == 'j') for (int Node = 8 * m; Node <= LastNode; Node++) Force[Node][0] += Load / (LastNode - 16 * m + 1);
    if (key == 'y') for (int Node = 8 * m; Node <= LastNode; Node++) Force[Node][1] -= Load / (LastNode - 16 * m + 1);
    if (key == 'h') for (int Node = 8 * m; Node <= LastNode; Node++) Force[Node][1] += Load / (LastNode - 16 * m + 1);
    if (key == 'k') for (int Node = 8 * m; Node <= LastNode; Node++) Force[Node][2] -= Load / (LastNode - 16 * m + 1);
    if (key == 'o') for (int Node = 8 * m; Node <= LastNode; Node++) Force[Node][2] += Load / (LastNode - 16 * m + 1);

    //    if(key=='t') println (TensionCoefficient);
    // Zoom in and out
    if (key == 'z') ScaleFactor *= 1.1;
    if (key == 'x') ScaleFactor /= 1.1;

    // Reset program
    if (key == 'n') setup();

    // Poke in
    if (key == 'p' || key == 'l') {
      int r = 1;
      if (key == 'l') r = -1;
      if (m % 2 == 0) {
        Force[CentralNode][2] += r * Load / 4;
        Force[CentralNode - 1][2] += r * Load / 4;
        Force[CentralNode - m][2] += r * Load / 4;
        Force[CentralNode - m - 1][2] += r * Load / 4;
      } else {
        Force[CentralNode][2] += r * Load;
      }
    }

    if (key == ' ') ResetMemberLength(16 * m, LastMember);

    if (key == 'r' && !Released) {
      boolean Releasable = false;
      if (ConstantCableLength) {
        if (Iteration > SetLengthTrigger) {
          ResetMemberLength(16 * m, LastMember);
          Releasable = true;
        } else println("To release, wait for the cables to relax and set their length until Iteration " + SetLengthTrigger);
      } else Releasable = true;
      if (Releasable) {
        Release();
        Released = true;
      }
    }
  }

  dX = X0 - (Coord[int(5.5 * float(m)) - 1][0] - Coord[int(7.5 * float(m)) - 1][0]);
  dY = Y0 - (Coord[int(6.5 * float(m)) - 1][1] - Coord[int(4.5 * float(m)) - 1][1]);
  dZ = Z0 - Coord[CentralNode][2];
  eX = dX / X0;
  eY = dY / Y0;
  eZ = dZ / Z0;

  // Start Relaxing
  if (Relaxable) {
    for (int Member = 0; Member <= LastMember; Member++) {
      // Tension            = T   = T0 + (EA/L0)*(L - L0)
      // TensionCoefficient = T/L = EA/L0 + (T0 - EA)/L
      TensionCoefficient[Member] = EA[Member] / L0[Member] + (T0[Member] - EA[Member]) / Length(Member);
      if (!CableCompressible && Member >= 16 * m && TensionCoefficient[Member] < 0) TensionCoefficient[Member] = 0;
      for (int xyz = 0; xyz <= 2; xyz++) {
        // ForceComponent = T/L * Delta[xyz]
        float ForceComponent = TensionCoefficient[Member] * (Coord[End[Member][1]][xyz] - Coord[End[Member][0]][xyz]);
        Force[End[Member][0]][xyz] += ForceComponent;
        Force[End[Member][1]][xyz] -= ForceComponent;
      }
    }

    for (int Node = 0; Node <= LastNode; Node++) {
      for (int xyz = 0; xyz <= 2; xyz++) {
        if (Fixed[Node][xyz] == 0) {
          Veloc[Node][xyz] = RelaxationRate * Veloc[Node][xyz] + Force[Node][xyz] / Stiffness[Node];
          Coord[Node][xyz] += Veloc[Node][xyz];
        }
      }
    }
  }
  // Finish Relaxing
  // Start Drawing
  fill(0, 255, 0, 255);
  text("Iteration        " + Iteration, PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("x Radius m       " + (WidestInnerRadius * xScale / pow(10, 3)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("y Radius m       " + (WidestInnerRadius * yScale / pow(10, 3)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("Circumference m  " + (Circumference / pow(10, 3)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("Area m2          " + (Area / pow(10, 6)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;

  text("Sum Truss L Km:  " + (TotalTrussLength / pow(10, 6)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("Sum Cable L Km:  " + (TotalCableLength / pow(10, 6)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("Scale Factor     " + ScaleFactor, PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("Load kN          " + (Load / pow(10, 3)), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("dY/dX            " + (dY / dX), PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("eX               " + eX, PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("eY               " + eY, PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  text("dZ               " + dZ, PTextX, PTextY + DTextY * TextCount);
  TextCount++;
  translate(float(width) / 2.0, float(height) / 2.0);
  ortho(-float(width) / 2.0, float(width) / 2.0, -float(height) / 2.0, float(height) / 2.0, -width, width);
  rotateX(-xRot);
  rotateZ(-zRot);
  translate(xTrans * cos(zRot) - yTrans * sin(zRot), yTrans * cos(zRot) + xTrans * sin(zRot), yTrans * sin(xRot));
  scale(ScaleFactor);
  smooth();
  //strokeWeight(1.0/ScaleFactor);//OPENGL
  strokeWeight(1.0); //P3D
  stroke(255, 255, 255, 100);
  for (int Member = 0; Member <= LastMember; Member++) {
    // Colour the force 
    switch (Colour) {
    case 'F':
      float ThisForceSquared = 0.0;
      for (int xyz = 0; xyz <= 2; xyz++) {
        ThisForceSquared += pow(Force[End[Member][0]][xyz] + Force[End[Member][1]][xyz], 2);
        ForceColour[xyz] = int(255 * (1 - abs(Force[End[Member][0]][xyz] + Force[End[Member][1]][xyz]) / MaxForce));
      }

      float ThisForce = sqrt(ThisForceSquared);

      if (ThisForce > MaxForce) MaxForce = ThisForce;
      stroke(ForceColour[0], ForceColour[1], ForceColour[2], 255);
      MaxForce = sqrt(MaxForce);
      break;
    case 'M':
      if (Member < 4 * m) stroke(InnerRing);
      else if (Member < 8 * m) stroke(OuterRing);
      else if (Member < 16 * m) stroke(Truss);
      else stroke(Cables);
      break;
    case 'A':
      if (Member == 0 || Member == 16 * m) stroke(200, 0, 0, 255);
      else stroke(255, 255, 255, 255);
      break;
    case 'C':
      if (TensionCoefficient[Member] < 0) stroke(200, 0, 0, 255);
      else stroke(255, 255, 255, 255);
      break;
    }
    // Draw the line
    line(Coord[End[Member][0]][0], Coord[End[Member][0]][1], Coord[End[Member][0]][2], Coord[End[Member][1]][0], Coord[End[Member][1]][1], Coord[End[Member][1]][2]);
  }

  // Finish Drawing
  if (OutputEnabled) {
    if (Iteration % WriteInterval == 0) {
      Output.println((float(Iteration) / float(ClosingTrigger)) + "," + eX + "," + eY + "," + dZ);
    }

    if (Iteration == ReleaseTrigger) {
      if (ConstantCableLength) ResetMemberLength(16 * m, LastMember);
      Release();
      Released = true;
    }

    if (Iteration >= ClosingTrigger - 1) {
      if (Variable == 0) {
        OutputProperties();
        Output.close();
        println("End of analysis.");
        noLoop();
      } else if (Variable == 1) {
        m += 10;
        if (m < 100) setup();
        Variable = 0;
      } else if (Variable == 2) {
        CableT0 += P0;
        if (CableT0 < 10 * P0) setup();
        Variable = 0;
      }
    }
  }

  // Keep count of iterations  
  Iteration++;
}
