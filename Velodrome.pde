//http://www.processing.org/
//This program uses Alistair Day's Dynamic Relaxation technique
//Simplified version of the Verlet algorithm
//import processing.opengl.*;
float[][] Coord;
float[][] Force;
float[][] Veloc;
float[] Stiffness;
float[] EA;
float[] T0;
float[] L0;
float[] Delta;
int [][] End;
int[][] Fixed;
int[] ForceColour;
float[] ForceInitialColour;
int m;
float ScaleFactor;
int LastMember,LastNode;
boolean Relaxable;
boolean Pokable;
boolean Releasable;
boolean ForceColoured;
boolean MemberColoured;
boolean FirstMemberColoured;
int PokeNode;
int Iteration;
float PokeForce;
float RelaxationRate;
color Truss;
color InnerRing;
color OuterRing;
color Cables;

void setup()
{
  //size(1200,750,OPENGL);
  size(1200,750,P3D);
  //size(int(0.9*float(screen.width)),int(0.9*float(screen.height)),P3D);
  smooth();
  textFont(createFont("Arial",24));
  frameRate(30);
  textMode(SCREEN);
  Relaxable           = true;
  Pokable             = true;
  Releasable          = true;  
  ForceColoured       = false;
  MemberColoured      = true;
  FirstMemberColoured = true;
  Truss      = color(200,0,0,255);
  InnerRing  = color(0,200,0,255);
  OuterRing  = color(0,0,200,255);
  Cables     = color(255,255,255,255);
  ForceInitialColour = new float[3];
  float[] ForceInitialColour = {255,0,0};
  ForceColour = new int[3];  
  m=5;//Must be even;
  LastNode=m*(m+8)-1; // Counting from zero
  PokeNode = int(float(m)*(8+float(m)/2)+(float(m)+1)%2*float(m)/2); // Node closest to the centre
  PokeForce = 50.0;  
  RelaxationRate = 0.98;
  Coord = new float[LastNode+1][3];
  Force = new float[LastNode+1][3];
  Veloc = new float[LastNode+1][3];
  Stiffness = new float[LastNode+1];
  Fixed = new int[LastNode+1][3];
  Delta = new float[3];
  LastMember=2*m*(m+9)-1; // Counting from zero
  EA = new float[LastMember+1];
  L0 = new float[LastMember+1];
  T0 = new float[LastMember+1];
  End = new int[LastMember+1][2];
  ScaleFactor=10;
  float a=float(height)/(1.5*2.0*ScaleFactor);
  float InitialRotation = PI/4*(5+1/float(m));//5*PI/4+PI/(4*m);//+6*PI/4;//Rotate by Theta/2 and then 270 degrees
  float HeightScale = 0.2;
  float xScale = 1.0;
  float yScale = 1.0;
  float InnerRingScale = 1-PI*pow(3,0.5)/4/float(m);
  // println(InnerRingScale);
  float Theta;
  {
    int Node=-1;
    // Internal Ring
    for(int i=0;i<4*m;i++)
    {
      Node++;
      Theta = 2*PI*i/(4*m)+InitialRotation;
      Coord[Node][0]=InnerRingScale*a*cos(Theta)/float(m/m)*xScale;//Position in x axis
      Coord[Node][1]=InnerRingScale*a*sin(Theta)/float(m/m)*yScale;//Position in y axis
      Coord[Node][2]=HeightScale*(pow(Coord[Node][0],2)-pow(Coord[Node][1],2))/a;//Position in z axis
    }    
    // External Ring
    for(int i=0;i<4*m;i++)
    {
      Node++;
      Theta = 2*PI*i/(4*m)+PI/(4*m)+InitialRotation;
      Coord[Node][0]=a*cos(Theta)/float(m/m)*xScale;//Position in x axis
      Coord[Node][1]=a*sin(Theta)/float(m/m)*yScale;//Position in y axis
      Coord[Node][2]=HeightScale*(pow(Coord[Node][0],2)-pow(Coord[Node][1],2))/a;//Position in z axis
      if ((i+1)%1==0) for (int xyz=0;xyz<=2;xyz++) Fixed[Node][xyz]=1;
    }
    
    // Cable Net
    for(int j=0;j<m;j++)
    {
      for(int i=0;i<m;i++)
      {
          Node++;
          Coord[Node][0]= a*(i-float(m-1)/2)/float(m); //Position in x axis
          Coord[Node][1]= a*(j-float(m-1)/2)/float(m);//(a*j)/m-2*m; //Position in y axis
          Coord[Node][2]=HeightScale*(pow(Coord[Node][0],2)-pow(Coord[Node][1],2))/a;//Position in z axis
//          println(Node+" has " + Coord[Node][0] + ", "+ Coord[Node][1]);
      }
    }
    println("Check on last node number: Should be " + Node + " and is "+ LastNode);
  }
  {
    float NetEA = 10;
    float NetT0 = -NetEA*a/float(m);
    float EdgeEA = 2000000.0*NetEA;
    float EdgeT0 = -EdgeEA*a/float(m);

    int Member = -1;

    // Internal Ring
    for(int i=0;i<4*m;i++)
    {
      Member++;
      End[Member][0]=i;
      if ((i+1)==4*m) End[Member][1]=0;
      else End[Member][1]=i+1;
      L0[Member] = Length(Member);
      EA[Member] = EdgeEA*L0[Member];
      T0[Member] = EdgeT0+EA[Member];
      println( EA[Member]/L0[Member]);      
    }

    // External Ring
    for(int i=0;i<4*m;i++)
    {
      Member++;
      End[Member][0]=4*m+i;
      if ((i+1)==4*m) End[Member][1]=4*m;
      else End[Member][1]=4*m+i+1;
      L0[Member] = Length(Member);
      EA[Member] = EdgeEA*L0[Member];
      T0[Member] = EdgeT0+EA[Member];
//      println("Connect " + End[Member][0] + " with "+ End[Member][1]);      
    }
    
    // Truss
    for(int i=0;i<4*m;i++)
    {
      // LHS Member
      Member++;
      End[Member][0]=i;
      End[Member][1]=i+4*m;
      L0[Member] = Length(Member);
      EA[Member] = EdgeEA*L0[Member];
      T0[Member] = EdgeT0+EA[Member];
//      println("Connect " + End[Member][0] + " with "+ End[Member][1]);
      // RHS Member
      Member++;
      if ((i+1)==4*m) End[Member][1]=0;
      else End[Member][0]=i+1;
      End[Member][1]=i+4*m;
      L0[Member] = Length(Member);
      EA[Member] = EdgeEA*L0[Member];
      T0[Member] = EdgeT0+EA[Member];
//      println("Connect " + End[Member][0] + " with "+ End[Member][1]);  
    }
    
    // Edge Cables
    for(int j=0;j<m;j++)
    {
      for(int i=0;i<m;i++)
      {
        if (j==0) {
          Member++;
          End[Member][0]=i;//Ring
          End[Member][1]=8*m+(m*j)+i;//Grid
          L0[Member] = Length(Member);
          EA[Member] = NetEA*L0[Member];
          T0[Member] = NetT0+EA[Member];         
//          println("Connect " + End[Member][0] + " with "+ End[Member][1]);
        }
        if (j==(m-1)) {
          Member++;
          End[Member][0]=3*m-i-1;//Ring
          End[Member][1]=8*m+(m*j)+i;//Grid
          L0[Member] = Length(Member);
          EA[Member] = NetEA*L0[Member];
          T0[Member] = NetT0+EA[Member];               
//          println("Connect " + End[Member][0] + " with "+ End[Member][1]);
        }      
        if (i==0) {
          Member++;
          End[Member][0]=4*m-(j+1);//Ring
          End[Member][1]=8*m+(m*j);//Grid
          L0[Member] = Length(Member);
          EA[Member] = NetEA*L0[Member];
          T0[Member] = NetT0+EA[Member];               
//          println("Connect " + End[Member][0] + " with "+ End[Member][1]);
        }
        if (i==(m-1)) {
          Member++;
          End[Member][0]=m+j;//Ring
          End[Member][1]=8*m+m*(j+1)-1;//Grid
          L0[Member] = Length(Member);
          EA[Member] = NetEA*L0[Member];
          T0[Member] = NetT0+EA[Member];               
//          println("Connect " + End[Member][0] + " with "+ End[Member][1]);
        }  
      }
    }

    // Cable Net  
    for(int j=0;j<m;j++)
    {
      for(int i=0;i<m;i++)
      {
        if(i<m-1){
          Member++;
          End[Member][0]=8*m+j*m+i;
          End[Member][1]=8*m+j*m+i+1;
          L0[Member] = Length(Member);
          EA[Member] = NetEA*L0[Member];
          T0[Member] = NetT0+EA[Member];               
//          println("Connect " + End[Member][0] + " with "+ End[Member][1]);
        }
        if(j<m-1){
          Member++;
          End[Member][0]=8*m+j*m+i;
          End[Member][1]=8*m+j*m+i+m;
          L0[Member] = Length(Member);
          EA[Member] = NetEA*L0[Member];
          T0[Member] = NetT0+EA[Member];                
//          println("Connect " + End[Member][0] + " with "+ End[Member][1]);
        }
      }
    }    
    
    println("Check on last member number: Should be " + Member + " and is "+ LastMember);
    LastMember = Member;
  }
  
  for(int Node=0;Node<=LastNode;Node++) Stiffness[Node]=0.0;
  
  for(int Member=0;Member<=LastMember;Member++)
  {
    Stiffness[End[Member][0]]+=EA[Member]/L0[Member];
    Stiffness[End[Member][1]]+=EA[Member]/L0[Member];
  }
}

float PTextX=50.0;
float PTextY=400.0;
float TextYInterval=30.0;
float TextX=PTextX;
float TextY=PTextY;
int TextCount;
float OriginalTextY=400.0;
float TextRegion=100.0;
int OverText=0;
float zRot=0.0,xRot=0.0;
float MyMouseX=0.0,MyMouseXpressed=0.0,MyMouseY=0.0,MyMouseYpressed=0.0;
float xTrans=0.0,yTrans=0.0;
float theta=0.0;
float MaxForce;
float ForceRatio;
boolean Fast = false;

void draw()
{
  TextCount=1;
  background(0,0,0);
  if(mousePressed)
  {
    MyMouseXpressed=mouseX;
    MyMouseYpressed=mouseY;
    if(OverText==1)
    {
      TextY+=MyMouseYpressed-MyMouseY;
      ScaleFactor*=1.0-0.01*(MyMouseYpressed-MyMouseY);
    }
    else
    {
      if(mouseButton==LEFT)
      {
        zRot+=(MyMouseXpressed-MyMouseX)/300.0;
        xRot+=(MyMouseYpressed-MyMouseY)/300.0;
      }
      else
      {
        xTrans+=(MyMouseXpressed-MyMouseX);
        yTrans+=(MyMouseYpressed-MyMouseY);
      }
      TextY=OriginalTextY;
    }
    MyMouseX=MyMouseXpressed;
    MyMouseY=MyMouseYpressed;
  }
  else
  {
    MyMouseX=mouseX;
    MyMouseY=mouseY;
    TextY=OriginalTextY;
    if((MyMouseX-TextX)*(MyMouseX-TextX)+(MyMouseY-TextY)*(MyMouseY-TextY)<TextRegion*TextRegion)
    {
      OverText=1;
      fill(200,0,0,255);
    }
    else
    {
      OverText=0;
      fill(255,255,255,255);
    }
  }
  for(int Node=0;Node<=LastNode;Node++)
  {
    for(int xyz=0;xyz<=2;xyz++)Force[Node][xyz]=0.0;
  }
  
  if (keyPressed){
    if(key=='f') frameRate (30);
    if(key=='s') frameRate (5);
  }  
  
  if (Pokable&&keyPressed){
    if(key=='p'){
      if (m%2==0) {
        Force[PokeNode][2]+=PokeForce/4;
        Force[PokeNode-1][2]+=PokeForce/4;
        Force[PokeNode-m][2]+=PokeForce/4;
        Force[PokeNode-m-1][2]+=PokeForce/4;
//        println(PokeNode+", "+(PokeNode-1)+", "+(PokeNode-m)+", "+(PokeNode-m-1)+" excited");
      } else {
        Force[PokeNode][2]+=PokeForce;
//        println(PokeNode+" excited");
      }
    }
  }
  
  if (Releasable&&keyPressed){
    if(key=='r'){
      for(int i=4*m;i<8*m;i++)
      {
        if ((i+1)%(4*m)==0) for (int xyz=0;xyz<=2;xyz++) Fixed[i][xyz]=1;
        else for (int xyz=0;xyz<=1;xyz++) Fixed[i][xyz]=0;
      }
    }
  }    

  // Force[][2]+=0*sin(Theta);
  // Theta+=PI/4;

  // Start Relaxing
  if (Relaxable){
    for(int Member=0;Member<=LastMember;Member++)
    {
      float LengthSq=0.0;
      //Tension = T = T0 + (EA/L0)*(L - L0)
      //TensionCoefficient = T/L = EA/L0 + (T0 - EA)/L
      float TensionCoefficient=EA[Member]/L0[Member]+(T0[Member]-EA[Member])/Length(Member);
//      if(T0_minus_EA[Member]!=0.0)TensionCoefficient+=T0_minus_EA[Member]/sqrt(LengthSq);
      for(int xyz=0;xyz<=2;xyz++)
      {
        float ForceComponent=TensionCoefficient*(Coord[End[Member][1]][xyz]-Coord[End[Member][0]][xyz]);
        Force[End[Member][0]][xyz]+=ForceComponent;
        Force[End[Member][1]][xyz]-=ForceComponent;
      }
    }
    
    for(int Node=0;Node<=LastNode;Node++)
    {
      for(int xyz=0;xyz<=2;xyz++)
      {
        if(Fixed[Node][xyz]==0)
        {
        println('.');
        RelaxationRate = 0.98;
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
  text("Zoom",TextX,TextY);
  fill(255,255,255,255);
  text("Iteration "+ Iteration,PTextX,PTextY+TextYInterval*TextCount);TextCount++;  
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
         ForceColour[xyz] = 255-255*int(pow(pow(Force[End[Member][0]][xyz]+Force[End[Member][1]][xyz],2),0.5)/MaxForce);
       }
      
       float ThisForce = pow(ThisForceSquared,0.5);

       if (ThisForce>MaxForce) MaxForce=ThisForce;
       stroke(ForceColour[0],ForceColour[1],ForceColour[2],255);
       MaxForce=pow(MaxForce,0.5);             
    }
       
    if (MemberColoured&&!ForceColoured){
      if (Member<4*m) stroke(InnerRing);
      else if (Member<8*m) stroke(OuterRing);
      else if (Member<16*m) stroke(Truss);
      else stroke(Cables);
    }
      
    // Code to determine the first member and the first member connecting to the cable    
    if (FirstMemberColoured&&ForceColoured&&!MemberColoured){
      if (Member==0||Member==16*m) stroke(200,0,0,255);
      else stroke(255,255,255,255);
    }
    line(Coord[End[Member][0]][0],Coord[End[Member][0]][1],Coord[End[Member][0]][2],
    Coord[End[Member][1]][0],Coord[End[Member][1]][1],Coord[End[Member][1]][2]);
  }
  // Finish Drawing
}

float Length(int Member){
  float x1 = Coord[End[Member][0]][0];
  float x2 = Coord[End[Member][1]][0];
  float y1 = Coord[End[Member][0]][1];
  float y2 = Coord[End[Member][1]][1];
  float z1 = Coord[End[Member][0]][2];
  float z2 = Coord[End[Member][1]][2];
  return sqrt(pow(x2-x1,2)+pow(y2-y1,2)+pow(z2-z1,2));
}
