const NumWheels = 4;

// Global Variables
var irobot, isensor: integer;
    t, w: double;
    lines: TStrings;
    UDP: TStream;
    ControlMode: string;
    state : integer;
    d: double;
    DM: MATRIX;
    
procedure LineDistance(N, VECTOR_A, VECTOR_P:MATRIX);
var
  A_P: MATRIX;
begin
  A_P := Msub (Vector_A, Vector_P);
  DM := Msub(A_P,MmultReal(N,Mgetv(Mmult(Mtran(A_P),N),0,0)));

  d := MallNorm(DM);
end;

procedure ControladorB();
var
  N, Vector_A, Vector_P, Vector_F: MATRIX;
  a: double;
  VNOM, Ka, Kd: integer;
  v, w, v1, v2: double;
begin
  VNOM := 10;
  Ka := 100;
  Kd := 20;

  SetRCValue(4,3, format('%.3g',[GetRobotX(0)]));
  SetRCValue(5,3, format('%.3g',[GetRobotY(0)]));

  Vector_A := RangeToMatrix(4, 1, 2, 1);
  Vector_F := RangeToMatrix(4, 2, 2, 1);
  Vector_P := RangeToMatrix(4, 3, 2, 1);

  N := MmultReal((Msub(Vector_F, Vector_A)), 1/(MallNorm(Msub(Vector_F, Vector_A))));
  
  LineDistance(N, VECTOR_A, VECTOR_P); // Cálculo da distância do robot à linha
  
  if (((GetRobotTheta(0)<0) AND (GetRobotTheta(0)>(-pi/2)) AND (ATan2(Mgetv(N, 1, 0),Mgetv(N,0,0))>0) AND (ATan2(Mgetv(N, 1, 0),Mgetv(N,0,0))<(pi/2))) OR ((GetRobotTheta(0)>0) AND (GetRobotTheta(0)<(pi/2)) AND (ATan2(Mgetv(N, 1, 0),Mgetv(N,0,0))<0) AND (ATan2(Mgetv(N, 1, 0),Mgetv(N,0,0))>(-pi/2)))) then begin
    a := GetRobotTheta(0) - ATan2(Mgetv(N, 1, 0),Mgetv(N,0,0));
  end
  else
    a := GetRobotTheta(0)+pi*(1-sign(GetRobotTheta(0))) - ATan2(Mgetv(N, 1, 0),Mgetv(N,0,0)) - pi*(1-sign(ATan2(Mgetv(N, 1, 0),Mgetv(N,0,0))));

  v := VNOM;
  w := Kd*d + Ka*a;

  v1 := v + w*0.1/2;
  v2 := v - w*0.1/2;

  SetAxisSpeedRef (irobot, 0, v1);
  SetAxisSpeedRef (irobot, 1, v2);
  
  SetRCValue (8, 2, format('%.3g',[v]))
  SetRCValue (9, 2, format('%.3g',[w]))
  SetRCValue (10, 2, format('%.3g',[a*180/pi]))
end;

procedure ControladorA();
var
  N, Vector_A, Vector_P, Vector_F: MATRIX;
  a: double;
  VNOM, WNOM, ka: integer;
  v, w, v1, v2: double;
begin
  VNOM := 10;
  WNOM := 50;
  ka := 50
  
  SetRCValue(4,3, format('%.3g',[GetRobotX(0)]));
  SetRCValue(5,3, format('%.3g',[GetRobotY(0)]));

  Vector_A := RangeToMatrix(4, 1, 2, 1);
  Vector_F := RangeToMatrix(4, 2, 2, 1);
  Vector_P := RangeToMatrix(4, 3, 2, 1);
  
  N := MmultReal((Msub(Vector_F, Vector_A)), 1/(MallNorm(Msub(Vector_F, Vector_A))));

  LineDistance(N, VECTOR_A, VECTOR_P);
  
  a := + GetRobotTheta(0) - ATan2(Mgetv(DM, 1, 0),Mgetv(DM,0,0));

  v := VNOM;
  w := ka*a;
  
  v1 := v + w*0.1/2;
  v2 := v - w*0.1/2;
  
  SetAxisSpeedRef (irobot, 0, v1);
  SetAxisSpeedRef (irobot, 1, v2);
  
  SetRCValue (8, 2, format('%.3g',[v]))
  SetRCValue (9, 2, format('%.3g',[w]))
   SetRCValue (10, 2, format('%.3g',[a*180/pi]))

end;

procedure FollowLine ();
var
  DM, N, A_P, Vector_A, Vector_P, Vector_F, F_P: MATRIX;
  d, a, FD: double;
  DMAX, DHIST, TolPosFinal: double;
  v, w, v1, v2: double;
begin

  DMAX := 0.1;
  DHIST := 0.1;
  TolPosFinal := 0.1;

  SetRCValue(4,3, format('%.3g',[GetRobotX(0)]));
  SetRCValue(5,3, format('%.3g',[GetRobotY(0)]));

  Vector_A := RangeToMatrix(4, 1, 2, 1);
  Vector_F := RangeToMatrix(4, 2, 2, 1);
  Vector_P := RangeToMatrix(4, 3, 2, 1);

  N := MmultReal((Msub(Vector_F, Vector_A)), 1/(MallNorm(Msub(Vector_F, Vector_A))));

  A_P := Msub (Vector_A, Vector_P);
  DM := Msub(A_P,MmultReal(N,Mgetv(Mmult(Mtran(A_P),N),0,0)));

  d := MallNorm(DM);

  F_P := Msub (Vector_F, Vector_P);    //
  FD := MallNorm(F_P);                 // Distancia posicao final
  
  case state of
    1: begin
        ControladorA();

        if (d < (DMAX + DHIST)) then
          state := 2;
       end;
    2: begin
        ControladorB();

        if (d > DMAX) then
          state := 1;
        if ( FD < TolPosFinal) then   //Tolerancia em relação a posicao final, se distancia<0.1 pára
          state := 3;
       end;
    3: begin
        SetAxisSpeedRef (irobot, 0, 0);
        SetAxisSpeedRef (irobot, 1, 0);    //Robot parado
        
        if (FD > TolPosFinal) then
          state := 2;             //Se pos final mudar volta estado anterior
       end;
   end;


   SetRCValue (7, 2, format('%d',[state]))

end;

procedure KeyControl(v: double);
var v1, v2: double;
begin

  v := 10;

  v1 := 0;
  v2 := 0;
  if keyPressed(VK_RIGHT) then begin
    v1 := +1;
    v2 := -1;
  end;

  if keyPressed(VK_LEFT) then begin
    v1 := v1 - 1;
    v2 := v2 + 1;
  end;

  if keyPressed(VK_UP) then begin
    v1 := v1 + 1;
    v2 := v2 + 1;
  end;

  if keyPressed(VK_DOWN) then begin
    v1 := v1 - 1;
    v2 := v2 - 1;
  end;

  v1 := v1*v;
  v2 := v2*v;
  SetAxisSpeedRef(irobot, 0, v1);
  SetAxisSpeedRef(irobot, 1, v2);
end;

procedure TrackControl(v, k: double);
var v1, v2, err, ys: double;
    P: TPoint3D;
begin
  P := GetSolidPos(irobot, isensor);
  if P.y > 0 then begin
    err := -P.x;
  end else if P.y > -0.25 then begin
    err := -P.x + 0.1;
  end else begin
    err := -P.x;
  end;
  
  v1 := v - k * err;
  v2 := v + k * err;

  SetAxisSpeedRef(irobot, 0, v1);
  SetAxisSpeedRef(irobot, 1, v2);
end;


procedure Control;
var ref: double;
    s: string;
    odo1,odo2: integer;
    sens1,sens2:double;
begin

  if keyPressed(ord('R')) then begin
    SetRobotPos(irobot, 0, 0.4, 0, 0);
  end;

  if keyPressed(ord('S')) then begin
    ControlMode := 'keys';
  end else if keyPressed(ord('T')) then begin
    ControlMode := 'track';
  end else begin
    //ControlMode := 'keys';
  end;

  if ControlMode = 'keys' then begin
    KeyControl(10);
  end;
  
  t := t + 0.04;
  if w*t >= 2*pi then begin
    t := t - 2*pi/w;
  end;
  
  if controlMode = 'track' then begin
    TrackControl(10, 150);
  end;
  
  sens1:=GetSensorVal(0,0);
  sens2:=GetSensorVal(0,1);

  
  //GetAxisOdo(RobotIndex, AxisIndex);
  odo1:=GetAxisOdo(0,0);
  odo2:=GetAxisOdo(0,1);
  
  SetRCValue(1, 1 ,format('%.3g',[sens1]));
  SetRCValue(2, 1 ,format('%.3g',[sens2]));

  FollowLine();

end;


procedure Initialize;
begin
  irobot := 0;
  isensor := GetSolidIndex(irobot, 'NXTLightSensor');
  
  t := 0;
  w := 1;
  
  ControlMode := 'keys';
  state := 1;

end;
