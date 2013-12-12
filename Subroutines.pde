float Length(int Member){
  float LengthSq = 0.0;
  for(int xyz=0;xyz<=2;xyz++)
    LengthSq+= pow(Coord[End[Member][1]][xyz]-Coord[End[Member][0]][xyz],2);
  return sqrt(LengthSq);
}

float TotalMemberLength(int StartMember, int EndMember){
  float TotalLength = 0.0;
  for(int Member=StartMember;Member<=EndMember;Member++) TotalLength+= L0[Member];
  return TotalLength;
}

void CalculateMemberLength(){
  TotalTrussLength = TotalMemberLength(0,16*m-1); 
  TotalCableLength = TotalMemberLength(16*m,LastMember); 
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
  CalculateMemberLength(); 
}

void Release(){
    for(int i=0;i<8*m;i++) 
      for (int xyz=0;xyz<=2;xyz++) Fixed[i][xyz]=0; // Release all
    for(int i=0;i<8*m;i++) 
      for (int xyz=2;xyz<=2;xyz++) Fixed[i][xyz]=1; // Fix all in z direction
}

boolean FileExists(int TemporaryFile){
  File f = new File(dataPath(FileName+TemporaryFile+FileExtension));
  if (f.exists()) return true;
  else return false;
}

void OutputProperties(){
  if(OutputEnabled){
    Output.println("m,"+m);
    Output.println("Variable,"+Variable);
    Output.println("TrussRadius,"+TrussRadius);
    Output.println("TrussThickness,"+TrussThickness);
    Output.println("InnerTrussLength,"+InnerTrussLength);
    Output.println("TrussE,"+TrussE);
    Output.println("CableE,"+CableE);
    Output.println("TrussA,"+TrussA);
    Output.println("CableA,"+CableA);
    Output.println("TrussEA,"+TrussEA);
    Output.println("CableEA,"+CableEA);
    Output.println("TrussT0,"+TrussT0);
    Output.println("CableT0,"+CableT0);
    Output.println("ConstantStiffness,"+ConstantStiffness);
    Output.println("EdgeTangentCable,"+EdgeTangentCable);
  }
}
