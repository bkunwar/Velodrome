//http://www.processing.org/
//This program uses Alistair Day's Dynamic Relaxation technique
//Simplified version of the Verlet algorithm
//import processing.opengl.*;
      
int m = 10,
    CentralNode = int(float(m)*(8+float(m)/2)+(float(m)+1)%2*float(m)/2), // Node closest to the centre
    Iteration,
    n = int(m/2), // m over 2 rounded down which gives the number of extra cables on each side
    o = int((m-1)/2), // m minus 1 over 2 rounded down
    LastNode=m*(m+8)+(4*n)*(m-n-1)-1, // Counting from zero
    LastMember=2*m*(m+9)+4*o*(m-o)-1; // Counting from zero

int[] ForceColour = new int[3];
      
int [][] End = new int[LastMember+1][2], 
         Fixed = new int[LastNode+1][3];
          
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
      LongestSpan,
      RelaxationRate = 0.98,     
      Load = 100.00,        
      dX,
      dY,
      dZ,
      X0,
      Y0,
      Z0,
      Theta,
      ScaleFactor,
      InitialRotation = PI/4*(5+1/float(m)),        // Rotate by Theta/2 and then 270 degrees 5*PI/4+PI/(4*m)
      OuterRingScale = 1+PI*sqrt(3)/4.0/float(m);  // Renders the internal truss members the same length as the external members      
      
float[] Stiffness = new float[LastNode+1], 
        EA = new float[LastMember+1], 
        T0 = new float[LastMember+1], 
        L0 = new float[LastMember+1], 
        Delta = new float[3];

float[][] Coord, 
          Force, 
          Veloc;
    
boolean Relaxable = true, 
        ForceColoured = false, 
        MemberColoured = false, 
        FirstMemberColoured = true,
        Fast = false,
        Released = false;

color Truss      = color(200,0,0,255),
      InnerRing  = color(0,200,0,255),
      OuterRing  = color(0,0,200,255),
      Cables     = color(255,255,255,255);

void setup(){
  // size(1200,750,OPENGL);
  // size(int(0.9*float(screen.width)),int(0.9*float(screen.height)),P3D);
  size(1200,750,P3D);
  smooth();
  textFont(createFont("Courier New",20));
  frameRate(30);
  textMode(SCREEN);
  
  Coord = new float[LastNode+1][3]; 
  Force = new float[LastNode+1][3];
  Veloc = new float[LastNode+1][3];

  zRot = 0.0;
  xRot = 0.0;
  xTrans = 0.0;
  yTrans = 0.0;  

  float xScale = 1.0,
        yScale = 1.0,
        zScale = 0.5,
        NetScale = 1/sqrt(2)*(1-PI/4/float(m)); // To make the net fit within the truss ring
        
  LongestSpan = 2000.0;
  ScaleFactor = float(height)/(3.0*LongestSpan);
        
  float NetEA = 400.0/float(m),
        NetT0 = 1.0*NetEA,
        EdgeEA = pow(10,4)*NetEA,
        EdgeT0 = 0;              //-EdgeEA/float(m),
  
  Iteration = 0;

  {
    int Node=-1;
    
    // Internal Ring
    for(int i=0;i<4*m;i++)
    {
      Node++;
      Theta = 2*PI*i/(4*m)+InitialRotation;
      Coord[Node][0]=xScale*LongestSpan*cos(Theta);//Position in x axis
      Coord[Node][1]=yScale*LongestSpan*sin(Theta);//Position in y axis
      if ((i+1)%1==0) for (int xyz=0;xyz<=2;xyz++) Fixed[Node][xyz]=1;
    }
    
    // External Ring
    for(int i=0;i<4*m;i++)
    {
      Node++;
      Theta = 2*PI*i/(4*m)+PI/(4*m)+InitialRotation;
      Coord[Node][0]=xScale*OuterRingScale*LongestSpan*cos(Theta);//Position in x axis
      Coord[Node][1]=yScale*OuterRingScale*LongestSpan*sin(Theta);//Position in y axis
      if ((i+1)%1==0) for (int xyz=0;xyz<=2;xyz++) Fixed[Node][xyz]=1;
    }
    
    // Cable Net
    for(int j=0;j<m;j++)
    {
      for(int i=0;i<m;i++)
      {
          Node++;
          Coord[Node][0]=Coord[i][0]; //Position in x axis
          Coord[Node][1]=Coord[m+j][1]; //Position in y axis
          /*
          Coord[Node][0]=xScale*NetScale*LongestSpan*(i-float(m-1))/Divider; //Position in x axis
          Coord[Node][1]=yScale*NetScale*LongestSpan*(j-float(m-1))/Divider; //Position in y axis
          */
      }
    }

    for(int k=0;k<4;k++)
    {
      for(int j=1;j<=n;j++)
      {
        for(int i=j;i<m-j;i++)
        {
          Node++;
          int xNode = (i+m*k)*((k+1)%2)+(j-1+m*k)*(k%2);
          int yNode = (i+m*k)*(k%2)+(j-1+m*k)*((k+1)%2);
          Coord[Node][0]=Coord[xNode][0]; //Position in x axis
          Coord[Node][1]=Coord[yNode][1]; //Position in y axis          
          //println("Node "+Node+": "+xNode+" "+yNode);
          //println(Node+": "+(i)+" "+(j-1)+" "+k);
        }
      }
    }
    
    println("Check on last node number: Should be " + Node + " and is "+ LastNode);
    
    for (Node=0;Node<=LastNode;Node++){
      Coord[Node][2]=zScale*(pow(Coord[Node][0],2)-pow(Coord[Node][1],2))/LongestSpan;//Position in z axis
    }
  }
  {
    int Member = -1;

    // Internal Ring
    for(int i=0;i<4*m;i++)
    {
      Member++;
      End[Member][0]=i;
      if ((i+1)==4*m) End[Member][1]=0;
      else End[Member][1]=i+1;     
    }

    // External Ring
    for(int i=0;i<4*m;i++)
    {
      Member++;
      End[Member][0]=4*m+i;
      if ((i+1)==4*m) End[Member][1]=4*m;
      else End[Member][1]=4*m+i+1;
    }
    
    // Truss
    for(int i=0;i<4*m;i++)
    {
      // LHS Member
      Member++;
      End[Member][0]=i;
      End[Member][1]=i+4*m;

      // RHS Member
      Member++;
      if ((i+1)==4*m) End[Member][1]=0;
      else End[Member][0]=i+1;
      End[Member][1]=i+4*m;
    }
    
/*
    // Edge Cables
    for(int j=0;j<m;j++)
    {
      for(int i=0;i<m;i++)
      {
        if (j==0) {
          Member++;
          End[Member][0]=i;//Ring
          End[Member][1]=8*m+(m*j)+i;//Grid        
        }
        if (j==(m-1)) {
          Member++;
          End[Member][0]=3*m-i-1;//Ring
          End[Member][1]=8*m+(m*j)+i;//Grid            
        }      
        if (i==0) {
          Member++;
          End[Member][0]=4*m-(j+1);//Ring
          End[Member][1]=8*m+(m*j);//Grid           
        }
        if (i==(m-1)) {
          Member++;
          End[Member][0]=m+j;//Ring
          End[Member][1]=8*m+m*(j+1)-1;//Grid             
        }  
      }
    }
 */
 
    for(int k=0;k<4;k++)
    {
      int ThisSide=m*(m+8)+k*n*(m-n-1)-1;
      for(int j=1;j<=o;j++)
      {
        Member++;
        End[Member][0]=k*m+j-1;
        for(int i=j;i<m-j;i++)
        {
          ThisSide++;
          End[Member][1]=ThisSide;
          //println("Connect "+End[Member][0]+" "+End[Member][1]);
          Member++;
          End[Member][0]=End[Member-1][1];
          //println(Member+": "+(ThisSide+i*j)+" "+i+" "+(j)+" "+k);
        }
        End[Member][1]=k*m+m-j;
        //println("Connect "+End[Member][0]+" "+End[Member][1]);        
      }
    }
    
    for(int k=0;k<4;k++)
    {
      int ThisSide=m*(m+8)+k*n*(m-n-1)-1;
      for(int j=1;j<=o;j++)
      {
        for(int i=j;i<m-j;i++)
        {
          ThisSide++;
          if (i==j||i==m-j-1){
            Member++;            
            End[Member][0]=m*k+i;
            End[Member][1]=ThisSide;
            //println("Connect "+End[Member][0]+" "+End[Member][1]);            
          } else {
            Member++;            
            End[Member][0]=ThisSide;
            End[Member][1]=ThisSide+(m-1)-j*2;
            //println("Connect "+End[Member][0]+" "+End[Member][1]);            
           
          }
        }     
      }
    }    
    
    /*
    for(int k=0;k<4;k++)
    {
      int ThisSide=m*(m+8)+k*n*(m-n-1)-1;
      for(int j=1;j<m-1;j++)
      {
        Member++;
        End[Member][0]=m*k+j;
        End[Member][1]=ThisSide+j;
        //println("Connect "+End[Member][0]+" "+End[Member][1]);   
        for(int i=j;i<m-j;i++)
        {
          //Member++;
          //ThisSide++;
          //println(Member+": "+ThisSide+" "+i+" "+j+" "+k);
        }
        //End[Member][1]=k*m+m-j;
        //println("Connect "+End[Member][0]+" "+End[Member][1]);        
      }
    }  
    */

    // Inner Cable Net
    for(int j=0;j<m;j++)
    {
      for(int i=0;i<m;i++)
      {
        if(i<m-1){
          Member++;
          End[Member][0]=8*m+j*m+i;
          End[Member][1]=8*m+j*m+i+1;            
//          println("Connect " + End[Member][0] + " with "+ End[Member][1]);
        }
        if(j<m-1){
          Member++;
          End[Member][0]=8*m+j*m+i;
          End[Member][1]=8*m+j*m+i+m;
//          println("Connect " + End[Member][0] + " with "+ End[Member][1]);
        }
      }
    }
    
    println("Check on last member number: Should be " + Member + " and is "+ LastMember);
    LastMember = Member;    
    
    
    
    for (Member=0;Member<=LastMember;Member++){
      if (Member<16*m){
        L0[Member] = Length(Member);              
        EA[Member] = EdgeEA;
        T0[Member] = EdgeT0;
      } else {
        L0[Member] = Length(Member);
        EA[Member] = NetEA;
        T0[Member] = NetT0;
      }
    }
  }
  
  ResetNodeStiffness(0,LastMember);
  
  X0 = Coord[int(5.5*float(m))-1][0]-Coord[int(7.5*float(m))-1][0];
  Y0 = Coord[int(6.5*float(m))-1][1]-Coord[int(4.5*float(m))-1][1];
  Z0 = Coord[CentralNode][2];
}

//-------------
// END OF SETUP
//-------------

void draw() {
  int TextCount = 0;
  background(0,0,0);
  if(mousePressed) {
    MyMouseXpressed=mouseX;
    MyMouseYpressed=mouseY;
    if(mouseButton==LEFT) {
      zRot+=(MyMouseXpressed-MyMouseX)/300.0;
      xRot+=(MyMouseYpressed-MyMouseY)/300.0;
    } else {
      xTrans+=(MyMouseXpressed-MyMouseX);
      yTrans+=(MyMouseYpressed-MyMouseY);
    }
    MyMouseX=MyMouseXpressed;
    MyMouseY=MyMouseYpressed;
  } else {
    MyMouseX=mouseX;
    MyMouseY=mouseY;
  }
  
  for(int Node=0;Node<=LastNode;Node++)
    for(int xyz=0;xyz<=2;xyz++)Force[Node][xyz]=0.0;

  if (keyPressed){
    // Increase or decrease frame rate
    if(key=='f') frameRate (30);
    if(key=='d') frameRate (5);
    
    // Increase or decrease poke force
    if(key=='a') Load*=1.1;
    if(key=='s') Load/=1.1;
    
    // UDL in x, y or z direction 

    if(key=='g') for(int Node=8*m;Node<=LastNode;Node++) Force[Node][0]-=Load/pow(m,2);
    if(key=='j') for(int Node=8*m;Node<=LastNode;Node++) Force[Node][0]+=Load/pow(m,2);
    if(key=='y') for(int Node=8*m;Node<=LastNode;Node++) Force[Node][1]-=Load/pow(m,2);
    if(key=='h') for(int Node=8*m;Node<=LastNode;Node++) Force[Node][1]+=Load/pow(m,2);
    if(key=='k') for(int Node=8*m;Node<=LastNode;Node++) Force[Node][2]-=Load/pow(m,2);
    if(key=='o') for(int Node=8*m;Node<=LastNode;Node++) Force[Node][2]+=Load/pow(m,2);    
    
//    if(key=='t') println (TensionCoefficient);
    
    // Zoom in and out
    if(key=='z') ScaleFactor*=1.1;
    if(key=='x') ScaleFactor/=1.1;

    // Reset program
    if(key=='n') setup();
    
    // Poke in
    if(key=='p'||key=='l'){
      int r = 1;
      if (key=='l') r = -1;       
      if (m%2==0) {
        Force[CentralNode][2]+=r*Load/4;
        Force[CentralNode-1][2]+=r*Load/4;
        Force[CentralNode-m][2]+=r*Load/4;
        Force[CentralNode-m-1][2]+=r*Load/4;
      } else {
        Force[CentralNode][2]+=r*Load;
      }
    }
    
    if(key==' ') ResetMemberLength(16*m,LastMember);
    
    if(key=='r'){
      for(int i=0;i<8*m;i++) 
        for (int xyz=0;xyz<=2;xyz++) Fixed[i][xyz]=0; // Release all
      for(int i=0;i<8*m;i++) 
        for (int xyz=2;xyz<=2;xyz++) Fixed[i][xyz]=1; // Fix all in z direction
    }
  }
  
  dX = X0 - (Coord[int(5.5*float(m))-1][0]-Coord[int(7.5*float(m))-1][0]);
  dY = Y0 - (Coord[int(6.5*float(m))-1][1]-Coord[int(4.5*float(m))-1][1]);
  dZ = Z0 - Coord[CentralNode][2];

  // Start Relaxing
  if (Relaxable){
    for(int Member=0;Member<=LastMember;Member++)
    {
      // Tension            = T   = T0 + (EA/L0)*(L - L0)
      // TensionCoefficient = T/L = EA/L0 + (T0 - EA)/L
      float TensionCoefficient = EA[Member]/L0[Member]+(T0[Member]-EA[Member])/Length(Member);
      for(int xyz=0;xyz<=2;xyz++)
      {
        // ForceComponent = T/L * Delta[xyz]
        float ForceComponent = TensionCoefficient*(Coord[End[Member][1]][xyz]-Coord[End[Member][0]][xyz]);
        Force[End[Member][0]][xyz] += ForceComponent;
        Force[End[Member][1]][xyz] -= ForceComponent;
      }
    }
    
    for(int Node=0;Node<=LastNode;Node++){
      for(int xyz=0;xyz<=2;xyz++){
        if(Fixed[Node][xyz]==0){
        Veloc[Node][xyz]=RelaxationRate*Veloc[Node][xyz]+Force[Node][xyz]/Stiffness[Node];
        Coord[Node][xyz]+=Veloc[Node][xyz];
        }
      }
    }
  }
  // Finish Relaxing
  
  // Keep count of iterations
  Iteration++;

  // Start Drawing
  fill(0,255,0,255);
  text("Iteration    "+Iteration,PTextX,PTextY+DTextY*TextCount);TextCount++;
  text("Scale Factor "+ScaleFactor,PTextX,PTextY+DTextY*TextCount);TextCount++;
  text("Load         "+Load,PTextX,PTextY+DTextY*TextCount);TextCount++;
  text("dX           "+dX,PTextX,PTextY+DTextY*TextCount);TextCount++;  
  text("dY           "+dY,PTextX,PTextY+DTextY*TextCount);TextCount++;
  text("dZ           "+dZ,PTextX,PTextY+DTextY*TextCount);TextCount++;    
  text("dY/dX        "+(dY/dX),PTextX,PTextY+DTextY*TextCount);TextCount++;  
  translate(float(width)/2.0,float(height)/2.0);
  ortho(-float(width)/2.0,float(width)/2.0,-float(height)/2.0,float(height)/2.0,-width,width);
  rotateX(-xRot);
  rotateZ(-zRot);
  translate(xTrans*cos(zRot)-yTrans*sin(zRot),yTrans*cos(zRot)+xTrans*sin(zRot),yTrans*sin(xRot));
  scale(ScaleFactor);
  smooth();
  //strokeWeight(1.0/ScaleFactor);//OPENGL
  strokeWeight(1.0);//P3D
  stroke(255,255,255,100);  
  for(int Member=0;Member<=LastMember;Member++)
  {
    // Colour the force 
    if (ForceColoured){
       float ThisForceSquared=0.0;
       for(int xyz=0;xyz<=2;xyz++){
         ThisForceSquared += pow(Force[End[Member][0]][xyz]+Force[End[Member][1]][xyz],2);
         ForceColour[xyz] = int(255*(1-abs(Force[End[Member][0]][xyz]+Force[End[Member][1]][xyz])/MaxForce));
       }
      
       float ThisForce = sqrt(ThisForceSquared);

       if (ThisForce>MaxForce) MaxForce=ThisForce;
       stroke(ForceColour[0],ForceColour[1],ForceColour[2],255);
       MaxForce=sqrt(MaxForce);             
    }
       
    if (MemberColoured&&!ForceColoured){
      if (Member<4*m) stroke(InnerRing);
      else if (Member<8*m) stroke(OuterRing);
      else if (Member<16*m) stroke(Truss);
      else stroke(Cables);
    }
      
    // Code to determine the first member and the first member connecting to the cable    
    if (FirstMemberColoured&&!ForceColoured&&!MemberColoured){
      if (Member==0||Member==16*m) stroke(200,0,0,255);
      else stroke(255,255,255,255);
    }
    line(Coord[End[Member][0]][0],Coord[End[Member][0]][1],Coord[End[Member][0]][2],
         Coord[End[Member][1]][0],Coord[End[Member][1]][1],Coord[End[Member][1]][2]);
  }
  // Finish Drawing
}

float Length(int Member){
  float LengthSq = 0.0;
  for(int xyz=0;xyz<=2;xyz++)
    LengthSq+= pow(Coord[End[Member][1]][xyz]-Coord[End[Member][0]][xyz],2);
  return sqrt(LengthSq);
}

void ResetNodeStiffness(int StartMember, int EndMember){
  for(int Member=StartMember;Member<=EndMember;Member++)
  {
    Stiffness[End[Member][0]]+=EA[Member]/L0[Member];
    Stiffness[End[Member][1]]+=EA[Member]/L0[Member];
  }
}

void ResetMemberLength(int StartMember, int EndMember){
  for(int Member=StartMember;Member<=EndMember;Member++) L0[Member] = Length(Member);
  ResetNodeStiffness (StartMember, EndMember);
}

