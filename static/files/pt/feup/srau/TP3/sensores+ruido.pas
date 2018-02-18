const NumWrheels = 4;

// Global Variables
var irobot, isensor: integer;
    t, w: double;
    lines: TStrings;
    UDP: TStream;
    ControlMode: string;
    state, state_path: integer;
    DM: MATRIX;
    d: double;
    x_atual, y_atual, theta: double;
    sensorFront, sensorBack: double;
    ObsAng, dw: double;
    marco : integer;

procedure AssociationModule ();
begin

    if ((x_Atual > 0) AND (y_Atual > 0)) then
        marco := 1;
    if ((x_Atual > 0) AND (y_Atual < 0)) then
        marco := 2;
    if ((x_Atual < 0) AND (y_Atual < 0)) then
        marco := 3;
    if ((x_Atual < 0) AND (y_Atual > 0)) then
        marco := 4;
    if ((sensorFront < 0) AND (sensorBack < 0)) then
        marco := 0;
end;

procedure ObservationModule();
var
  L, Lw: double;
begin

  L := 0.1;  // sensors distance
  Lw := 0.5 + 0.2 - 0.025; //mark distance to square middle line

  ObsAng := aTan2 ((sensorBack - sensorFront), L);
  dw := ((sensorFront + sensorBack)/2)*Cos(ObsAng);

  SetRCValue(2,6, format('%.3g', [ObsAng]));
  SetRCValue(3,6, format('%.3g', [dw]));

  if ((marco = 1) AND (dw > 0) AND (ObsAng > 0)) then begin
    x_atual :=  Lw - dw;
    theta := -pi()/2 + ObsAng;
  end;

  if ((marco = 2) AND (dw > 0) AND (ObsAng > 0)) then begin
    y_atual :=  -(Lw - dw);
    theta := -pi() + ObsAng;
  end;

  if ((marco = 3) AND (dw > 0) AND (ObsAng > 0)) then begin
    x_atual :=  -(Lw - dw);
    theta := pi()/2 + ObsAng;
  end;

  if ((marco = 4) AND (dw > 0)) then begin
    y_atual :=  Lw - dw;
    theta := ObsAng;
  end;

  if theta > (Pi()) then     //adjust angle between -pi and pi;
    theta := theta - 2*Pi();
  if theta < (-Pi()) then
    theta := theta + 2*Pi();

  SetRCValue(6,4, format('%.3g', [x_atual]));
  SetRCValue(7,4, format('%.3g', [y_atual]));
  SetRCValue(8,4, format('%.3g', [theta*180/Pi()]));

end;

procedure EstimationModule();

begin



end;

procedure Odometria(odo2, odo1:double);
var
  d_at:double;
  delta_theta:double;
begin
  d_at := (Odo1 + Odo2)/2*0.0005;
  delta_theta := (Odo1 - Odo2)*0.0005/0.1;

  theta := theta + delta_theta;

  if theta > (Pi()) then
    theta := theta - 2*Pi();

  if theta < (-Pi()) then
    theta := theta + 2*Pi();


  x_atual := x_atual + d_at*cos(theta + delta_theta/2);
  y_atual := y_atual + d_at*sin(theta + delta_theta/2);

  SetRCValue(6,4, format('%.3g', [x_atual]));
  SetRCValue(7,4, format('%.3g', [y_atual]));
  SetRCValue(8,4, format('%.3g', [theta*180/Pi()]));
end;

procedure LineDistance(N, VECTOR_A, VECTOR_P:MATRIX);
var
  A_P: MATRIX;
  aux: double;
begin
  A_P := Msub (Vector_A, Vector_P);
  DM := Msub(A_P,MmultReal(N,Mgetv(Mmult(Mtran(A_P),N),0,0)));

  aux := Sign(MGetV(N, 0, 0)*MGetV(DM, 1, 0) - MGetV(N, 1, 0)*MGetV(DM, 0, 0));
  d := aux * MallNorm(DM);
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

  Vector_A := RangeToMatrix(6, 2, 2, 1);
  Vector_F := RangeToMatrix(6, 3, 2, 1);
  Vector_P := RangeToMatrix(6, 4, 2, 1);

  N := MmultReal((Msub(Vector_F, Vector_A)), 1/(MallNorm(Msub(Vector_F, Vector_A))));

  LineDistance(N, VECTOR_A, VECTOR_P); // Robot line distance

  if (((theta<0) AND (theta>(-pi/2)) AND (ATan2(Mgetv(N, 1, 0),Mgetv(N,0,0))>0) AND (ATan2(Mgetv(N, 1, 0),Mgetv(N,0,0))<(pi/2))) OR ((theta>0) AND (theta<(pi/2)) AND (ATan2(Mgetv(N, 1, 0),Mgetv(N,0,0))<0) AND (ATan2(Mgetv(N, 1, 0),Mgetv(N,0,0))>(-pi/2)))) then begin
    a := theta - ATan2(Mgetv(N, 1, 0),Mgetv(N,0,0));
  end
  else
    a := theta+pi*(1-sign(theta)) - ATan2(Mgetv(N, 1, 0),Mgetv(N,0,0)) - pi*(1-sign(ATan2(Mgetv(N, 1, 0),Mgetv(N,0,0))));

  v := VNOM;
  w := Kd*d + Ka*a;

  v1 := v + w*0.1/2;
  v2 := v - w*0.1/2;

  SetAxisSpeedRef (irobot, 0, v1);
  SetAxisSpeedRef (irobot, 1, v2);

  SetRCValue (11, 5, format('%.3g',[v]));
  SetRCValue (12, 5, format('%.3g',[w]));
//  SetRCValue (10, 2, format('%.3g',[a*180/pi]));
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
    ka := 50;

  Vector_A := RangeToMatrix(6, 2, 2, 1);
  Vector_F := RangeToMatrix(6, 3, 2, 1);
  Vector_P := RangeToMatrix(6, 4, 2, 1);

  N := MmultReal((Msub(Vector_F, Vector_A)), 1/(MallNorm(Msub(Vector_F, Vector_A))));

  LineDistance(N, VECTOR_A, VECTOR_P);

  a := + theta - ATan2(Mgetv(DM, 1, 0),Mgetv(DM,0,0));

  v := VNOM;
  w := ka*a;

  v1 := v + w*0.1/2;
  v2 := v - w*0.1/2;

  SetAxisSpeedRef (irobot, 0, v1);
  SetAxisSpeedRef (irobot, 1, v2);

  SetRCValue (11, 5, format('%.3g',[v]));
  SetRCValue (12, 5, format('%.3g',[w]));
//  SetRCValue (10, 2, format('%.3g',[a*180/pi]));

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


  Vector_A := RangeToMatrix(6, 2, 2, 1);
  Vector_F := RangeToMatrix(6, 3, 2, 1);
  Vector_P := RangeToMatrix(6, 4, 2, 1);

  N := MmultReal((Msub(Vector_F, Vector_A)), 1/(MallNorm(Msub(Vector_F, Vector_A))));

  A_P := Msub (Vector_A, Vector_P);
  DM := Msub(A_P,MmultReal(N,Mgetv(Mmult(Mtran(A_P),N),0,0)));

  d := MallNorm(DM);

  F_P := Msub (Vector_F, Vector_P);    //
  FD := MallNorm(F_P);                 // Final Pose distance

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


   SetRCValue (12, 2, format('%d',[state]));

end;

procedure FollowPath ();
var
  xi, yi, xf, yf: double;

begin
  case state_path of
    1: begin
        xi := 0.5; yi := 0.5;
        xf := 0.5; yf := -0.5;

        // Escreve na matriz o ponto inicial
        SetRCValue(6,2, format('%.3g', [xi]));
        SetRCValue(7,2, format('%.3g', [yi]));

        // Escreve na matriz o ponto final
        SetRCValue(6,3, format('%.3g', [xf]));
        SetRCValue(7,3, format('%.3g', [yf]));

        SetRCValue (11, 2, format('%d',[state_path]));
        FollowLine ();
        if (y_atual < -0.4) then begin
          state_path := 2;
        end;
       end;
    2: begin
        xi := 0.5; yi := -0.5;
        xf := -0.5; yf := -0.5;

        // Escreve na matriz o ponto inicial
        SetRCValue(6,2, format('%.3g', [xi]));
        SetRCValue(7,2, format('%.3g', [yi]));

        // Escreve na matriz o ponto final
        SetRCValue(6,3, format('%.3g', [xf]));
        SetRCValue(7,3, format('%.3g', [yf]));

        SetRCValue (11, 2, format('%d',[state_path]));
        FollowLine ();

        if (x_atual < -0.4) then begin
          state_path := 3;
        end;
       end;
    3: begin
        xi := -0.5; yi := -0.5;
        xf := -0.5; yf := 0.5;

        // Escreve na matriz o ponto inicial
        SetRCValue(6,2, format('%.3g', [xi]));
        SetRCValue(7,2, format('%.3g', [yi]));

        // Escreve na matriz o ponto final
        SetRCValue(6,3, format('%.3g', [xf]));
        SetRCValue(7,3, format('%.3g', [yf]));

        SetRCValue (11, 2, format('%d',[state_path]));
        FollowLine ();

        if (y_atual > 0.4) then begin
          state_path := 4;
        end;
       end;
    4: begin
        xi := -0.5; yi := 0.5;
        xf := 0.5; yf := 0.51;

        // Escreve na matriz o ponto inicial
        SetRCValue(6,2, format('%.3g', [xi]));
        SetRCValue(7,2, format('%.3g', [yi]));

        // Escreve na matriz o ponto final
        SetRCValue(6,3, format('%.3g', [xf]));
        SetRCValue(7,3, format('%.3g', [yf]));

        SetRCValue (11, 2, format('%d',[state_path]));
        FollowLine ();

        if (x_atual > -0.4) then begin
          state_path := 1;
        end;
       end;
  end;


end;


////////////////////////////////////////////////////////////////////////////////


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
    mean, stddev, noiseFront, noiseBack: double;
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

  mean := 0;
  stddev := 0.002;
  noiseFront := RandG(mean, stddev);
  noiseBack := RandG(mean, stddev);
  SensorFront := GetSensorVal(0,0) + noiseFront;
  SensorBack := GetSensorVal(0,1) + noiseBack;


  //GetAxisOdo(RobotIndex, AxisIndex);
  odo1 := GetAxisOdo(0,0);
  odo2 := GetAxisOdo(0,1);

  Odometria(odo1, odo2);
  AssociationModule();
  if (marco <> 0) then
      ObservationModule();

  SetRCValue(1, 6 ,format('%d',[marco]));
  SetRCValue(2, 2 ,format('%.3g',[sensorFront]));
  SetRCValue(3, 2 ,format('%.3g',[sensorBack]));
  SetRCValue(5, 6 ,format('%.3g',[noiseFront]));
  SetRCValue(6, 6 ,format('%.3g',[noiseBack]));

  FollowPath();

end;


procedure Initialize;
begin
  irobot := 0;
  isensor := GetSolidIndex(irobot, 'NXTLightSensor');

  t := 0;
  w := 1;

  ControlMode := 'keys';

  theta := -Pi()/2;
  x_atual := 0.5;
  y_atual := 0.5;

  SetRCValue(6,4, format('%.3g', [x_atual]));
  SetRCValue(7,4, format('%.3g', [y_atual]));
  SetRCValue(8,4, format('%.3g', [theta*180/Pi()]));

  state := 1;
  state_path := 1;
  marco := 0;
end;
