const NumWheels = 4;



// Global Variables
var irobot, isensor: integer;
    t, w: double;
    lines: TStrings;
    UDP: TStream;
    ControlMode: string;
    state: integer;

procedure VelocidadeRodas (v, vw: double);
var
  v1, v2, b: double;

begin
  b := 0.1;

  v1 := v + vw*(b/2);
  v2 := v - vw*(b/2);
  
  SetRCValue(12, 1 ,'v1');
  SetRCValue(13, 1 ,'v2');
  SetRCValue(12, 2 ,format('%.3g',[v1]));
  SetRCValue(13, 2 ,format('%.3g',[v2]));

end;

// Procedure GoToXYTheta
procedure GOTOXYTheta (xf, yf, thetaf: double);
var
   x, y, theta: double;
   v1, v2: double;
   v, vw: double;
   b: double;
   aux_theta, aux_x, aux_y: double;
   WNOM, VNOM, GAIN_FWD, V_DA, GAIN_DA, W_DA: double;
   MAX_ETF, HIST_ETF, DIST_DA, DIST_NEWPOSE, TOL_FINDIST, TOL_FINTHETA, THETA_DA, THETA_NEWPOSE: double;
   
begin

  b := 0.1; // Distância entre rodas do robot
  
  // Valores nominais das velocidades lineares e angulares e ganhos
  WNOM := 20;
  W_DA := 5;
  VNOM := 10;
  V_DA := 0.25;
  GAIN_FWD := -0.5;
  GAIN_DA := 0.05;
  
  // Erros admitidos e tolerancias do sistema
  MAX_ETF := 0.1;
  HIST_ETF := 0.3;
  DIST_DA := 0.1;
  THETA_DA := 0.15;
  DIST_NEWPOSE := 0.25;
  TOL_FINDIST := 0.05;
  THETA_NEWPOSE := 0.21;
  TOL_FINTHETA := 0.01;
  

  // Aquisicao das posicoes do robot
  x := GetRobotX (0);
  y := GetRobotY (0);
  theta := GetRobotTheta (0);
  
  // Realiza uma mudança de referencial, para o referencial do robot.
  aux_x := xf - x;
  aux_y := yf - y;
  
  // Calcula o angulo do robot para a osição que pretende ir.
  aux_theta := aTan2 (aux_y, aux_x);

  // Instrucoes de Debug
  SetRCValue(3, 3 ,'Actual');
  SetRCValue(4, 3 ,format('%.3g',[x]));
  SetRCValue(5, 3 ,format('%.3g',[y]));
  SetRCValue(6, 3 ,format('%.3g',[theta]));

  SetRCValue(8, 1 ,'Erro Pos');
  SetRCValue(9, 1 ,'Erro Angulo');
  SetRCValue(10, 1 ,'Estado Actual');
  SetRCValue(8, 2 ,format('%.3g',[sqrt((aux_x*aux_x)+(aux_y*aux_y))]));
  SetRCValue(9, 2 ,format('%.3g',[abs(theta-thetaf)]));
  SetRCValue(10, 2 ,format('%d',[state]));
  /////////////////
  
  case state of
  1: begin // Rotacao para a posicao final
    v := 0;
    vw := 1*sign(theta-aux_theta) * WNOM;
    
    if (abs(aux_theta-theta) < MAX_ETF) then
      state := 2;
  end;
  
  2: begin // Move para a posicao final
    v := VNOM;
    vw := GAIN_FWD * (theta-aux_theta);
    
    if (sqrt((aux_x*aux_x)+(aux_y*aux_y)) < DIST_DA) then
      state := 3;
    if (abs(theta-aux_theta) > MAX_ETF + HIST_ETF) then
      state := 1;
  end;
  
  3: begin // Desacelaracao da velocidade linear ao aproximar da posicao final
    v := V_DA;
    vw := GAIN_DA * (theta-aux_theta);
    
    if (sqrt((aux_x*aux_x)+(aux_y*aux_y)) > DIST_NEWPOSE) then
      state := 1;
    if (sqrt((aux_x*aux_x)+(aux_y*aux_y)) < TOL_FINDIST) then
      state := 4;
  end;
  
  4: begin // Rotacao para theta final
    v := 0;
    vw := -1*sign(thetaf-theta)*WNOM;
    
    if (sqrt((aux_x*aux_x)+(aux_y*aux_y)) > DIST_NEWPOSE) then
      state := 1;
    if (abs(thetaf-theta) < THETA_DA) then
      state := 5;
  end;
  
  5: begin // Desaceleracao da velocidade angular ao aproximar de theta final
    v := 0;
    vw := -1*sign(thetaf-theta)*W_DA;
    
    if ((sqrt((aux_x*aux_x)+(aux_y*aux_y)) > DIST_NEWPOSE) OR (abs(thetaf-theta) > THETA_NEWPOSE)) then
      state := 1;
    if (abs(thetaf-theta) < TOL_FINTHETA) then
      state := 6;
  end;
  
  6: begin // Posicao Final
    v := 0;
    vw := 0;
    
    if ((sqrt((aux_x*aux_x)+(aux_y*aux_y)) > DIST_NEWPOSE) OR (abs(thetaf-theta) > THETA_NEWPOSE)) then
      state := 1;
  end;
  end;
  
  // Calculo da velocidade dos motores de cada roda
  VelocidadeRodas (v, vw);
  v1 := v + vw*b/2;
  v2 := v - vw*b/2;
  
  // Acionamento da velocidade dos motores
  SetAxisSpeedRef(irobot, 0, v1);
  SetAxisSpeedRef(irobot, 1, v2);
  
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
    x_f, y_f, theta_f: double;
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

  SetRCValue(4, 1 ,'X');
  SetRCValue(5, 1 ,'Y');
  SetRCValue(6, 1 ,'Theta');
  
  x_f := GetRCValue(4, 2);
  y_f := GetRCValue(5, 2);
  theta_f := GetRCValue(6, 2);
  
  GOTOXYTheta (x_f, y_f, theta_f);

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
