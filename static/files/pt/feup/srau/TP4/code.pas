// Global Variables Here

// Global Variables
var Vnom: double;
    xyzt_state: string;
    Target: TPoint3D;
    Target_Theta: double;
    state : integer;
    int_dist: double;
    state_path: integer;
    current_position: TPoint3D;

procedure ManualControl();
var V, W, Vz, V1, V2: double;
    Vmax, Wmax: double;
begin
  Vmax := 4;
  Wmax := 2;

  V := 0;
  W := 0;

  if KeyPressed(vk_down) then begin
    V := -1;
  end else if KeyPressed(vk_left) then begin
    W := 1;
  end else if KeyPressed(vk_right) then begin
    W := -1
  end else if KeyPressed(vk_up) then begin
    V := 1;
  end;

  V1 := V * Vmax - W * Wmax;
  V2 := V * Vmax + W * Wmax;

  if KeyPressed(ord('1')) then begin
    Vz := -0.5;
  end else if KeyPressed(ord('2')) then begin
    Vz := 0;
  end else if KeyPressed(ord('3')) then begin
    Vz := 0.5;
  end;

  SetAxisVoltageRef(0, 0, Vnom * V1);
  SetAxisVoltageRef(0, 1, Vnom * V2);
  SetAxisVoltageRef(0, 2, Vnom * Vz);
  SetAxisVoltageRef(0, 3, Vnom * Vz);

end;

procedure GotoXYZTheta();

var
    V, W, Vz, V1, V2, theta: double;
    aux_x, aux_y, aux_z, aux_theta, e_theta, dist, dist_sq: double;
    b: double;
    WNOM, VNOM, GAIN_FWD, V_DA, GAIN_DA, W_DA: double;
    HEIGHT_DA, TOL_HEIGHT, MAX_ETF, HIST_ETF, DIST_DA, DIST_NEWPOSE, TOL_FINDIST, TOL_FINTHETA, THETA_DA, THETA_NEWPOSE: double;
    Vmax, Wmax: double;
begin

   Vmax := 1;
   Wmax := 1;
   Vz := 0;

   b := 0.1; // Distância entre rodas do robot

   // Valores nominais das velocidades lineares e angulares e ganhos
   WNOM := 4;
   W_DA := 1;
   VNOM := 1;
   V_DA := -3;
   GAIN_FWD := 2;
   GAIN_DA := 1;

   // Erros admitidos e tolerancias do sistema
   MAX_ETF := 0.1;
   HIST_ETF := 0.2;
   DIST_DA := 1.5;
   THETA_DA := 0.15;
   DIST_NEWPOSE := 0.7;
   TOL_FINDIST := 1;
   THETA_NEWPOSE := 0.21;
   TOL_FINTHETA := 0.001;
   HEIGHT_DA := 0.2;
   TOL_HEIGHT := 0.1;
  
   current_position := GetSolidPos(0,0);
   theta := GetRobotTheta (0);
   
   
   target.x := GetRCValue(1,8);
   target.y := GetRCValue(2,8);
   target.z := GetRCValue(3,8);
   Target_Theta := GetRCValue(4,8);

   aux_x := target.x - current_position.x;
   aux_y := target.y - current_position.y;
   aux_z := target.z - current_position.z;

   // Calcula o angulo do robot para a posição que pretende ir.
   aux_theta := aTan2 (aux_y, aux_x);
   
   E_theta := DiffAngle(theta,aux_theta);
   dist_sq := aux_x*aux_x+aux_y*aux_y;
   dist := Sqrt(dist_sq);
   
   case state of
  1: begin // Rotacao para a posicao final
    v := 0;
    w := -1*sign(E_theta) * WNOM;
    vz := 0;

    if (abs(E_theta) < MAX_ETF) then
      state := 2;
  end;

  2: begin // Move para a posicao final
    v := VNOM * (dist);
    w := GAIN_FWD * (theta-aux_theta);
    vz := 0;

    if (dist < DIST_DA) then
      state := 3;
    if (abs(E_theta) > MAX_ETF + HIST_ETF) then
      state := 1;
  end;

  3: begin // Desacelaracao da velocidade linear ao aproximar da posicao final
    v := V_DA * (dist);
    w := GAIN_DA * (E_theta);
    vz := 0;
    
    int_dist := int_dist + dist*0.04;

    if (dist > DIST_NEWPOSE) then
      state := 1;
    if (dist < TOL_FINDIST) then
      state := 4;
  end;

  4: begin // Rotacao para theta final
    v := -0 * dist;
    w := 1*sign(DiffAngle(target_theta,theta))*WNOM;
    vz := 0;

    if (dist > DIST_NEWPOSE) then
      state := 1;
    if (abs(DiffAngle(target_theta,theta)) < THETA_DA) then
      state := 5;
  end;

  5: begin // Desaceleracao da velocidade angular ao aproximar de theta final
    v := 0 * dist;
    w := 1*sign(DiffAngle(theta,target_theta))*W_DA;
    vz := 0;

    if ((dist > DIST_NEWPOSE) OR (abs(DiffAngle(theta,target_theta)) > THETA_NEWPOSE)) then
      state := 1;
    if (abs(DiffAngle(target_theta,theta)) < TOL_FINTHETA) then
      state := 6;
  end;
  
  6: begin
    v := 0;
    w := 0;
    vz := aux_z * vnom;
    
    if (aux_z < HEIGHT_DA) then
      state := 7;
    if ((dist > DIST_NEWPOSE) OR (abs(DiffAngle(theta,target_theta)) > THETA_NEWPOSE)) then
      state := 1;
  end;

  7: begin
    v := 0;
    w := 0;
    vz := aux_z * GAIN_DA;
    
    if (aux_z < TOL_HEIGHT) then
      state := 8;
  end;

  8: begin // Posicao Final
    v := 0;
    w := 0;
    vz := 0;


    if ((dist > DIST_NEWPOSE) OR (abs(DiffAngle(target_theta,theta)) > THETA_NEWPOSE) OR (aux_z < TOL_HEIGHT)) then
      state := 1;
  end;
  end;
  
  V1 := V * Vmax - W * Wmax;
  V2 := V * Vmax + W * Wmax;
  
  SetAxisVoltageRef(0, 0, Vnom * V1);
  SetAxisVoltageRef(0, 1, Vnom * V2);
  SetAxisVoltageRef(0, 2, Vnom * Vz);
  SetAxisVoltageRef(0, 3, Vnom * Vz);
  
  SetRCValue(5,2, format('%.3g',[V1]));
  SetRCValue(5,3, format('%.3g',[V2]));
  SetRCValue(5,4, format('%.3g',[Vz]));
  
  SetRCValue(3,2, format('%.3g',[dist]));
  SetRCValue(3,3, format('%.3g',[theta-aux_theta]));
  SetRCValue(3,4, format('%.3g',[target_theta-theta]));
  
  SetRCValue(1,2, format('%d',[state]));
   
end;



procedure FollowPath ();
var
  xT, yT, zT, thetaT: double;

begin
  case state_path of
    1: begin
        xT := 0; yT := -2;
        zT := 2; thetaT := 0;

        SetRCValue(1,8, format('%.3g', [xT]));
        SetRCValue(2,8, format('%.3g', [yT]));
        SetRCValue(3,8, format('%.3g', [zT]));
        SetRCValue(4,8, format('%.3g', [thetaT]));
        SetRCValue (1, 10, format('%d',[state_path]));
        
        GotoXYZTheta();
        
        if (current_position.x > -0.8) then begin
          state_path := 2;
        end;
       end;
    2: begin
        xT := 0.5; yT := 4;
        zT := 2; thetaT := pi()/2.5;

        SetRCValue(1,8, format('%.3g', [xT]));
        SetRCValue(2,8, format('%.3g', [yT]));
        SetRCValue(3,8, format('%.3g', [zT]));
        SetRCValue(4,8, format('%.3g', [thetaT]));
        SetRCValue (1, 10, format('%d',[state_path]));

        GotoXYZTheta();

        if (current_position.y > 3) then begin
          state_path := 3;
        end;
       end;
    3: begin
        xT := 3; yT := 4;
        zT := 2; thetaT := -pi()/2;

        SetRCValue(1,8, format('%.3g', [xT]));
        SetRCValue(2,8, format('%.3g', [yT]));
        SetRCValue(3,8, format('%.3g', [zT]));
        SetRCValue(4,8, format('%.3g', [thetaT]));
        SetRCValue (1, 10, format('%d',[state_path]));

        GotoXYZTheta();

        if (current_position.x > 1.5) then begin
          state_path := 4;
        end;
       end;
    4: begin
        xT := 3; yT := -1;
        zT := 2; thetaT := -pi();

        SetRCValue(1,8, format('%.3g', [xT]));
        SetRCValue(2,8, format('%.3g', [yT]));
        SetRCValue(3,8, format('%.3g', [zT]));
        SetRCValue(4,8, format('%.3g', [thetaT]));
        SetRCValue (1, 10, format('%d',[state_path]));

        GotoXYZTheta();

        if (current_position.y < 0.5) then begin
          state_path := 5;
        end;
       end;
     5: begin
        xT := -3; yT := -1;
        zT := 2; thetaT := -pi();

        SetRCValue(1,8, format('%.3g', [xT]));
        SetRCValue(2,8, format('%.3g', [yT]));
        SetRCValue(3,8, format('%.3g', [zT]));
        SetRCValue(4,8, format('%.3g', [thetaT]));
        SetRCValue (1, 10, format('%d',[state_path]));

        GotoXYZTheta();
       end;
  end;
end;



// this procedure is called periodicaly (default: 40 ms)
procedure Control;
begin

//  ManualControl();
  FollowPath();

end;

// this procedure is called once when the script is started
procedure Initialize;
begin
  Vnom := 30;
  
  current_position := GetSolidPos(0,0);
  
  state := 1;
  state_path := 1;
  
  int_dist := 0;
  
  Target.x := 0;
  Target.y := 2;
  Target.z := -2;
  Target_Theta := Pi()/2;
  
  { Create sheet labels }
  SetRCValue(1,1, 'State');
  
  SetRCValue(3,1, 'Error');
  SetRCValue(2,2, 'Distance');
  SetRCValue(2,3, 'Angle');
  SetRCValue(2,4, 'Angle Final');
  
  SetRCValue(5,1, 'Velocity');
  SetRCValue(4,2, 'Motor 1');
  SetRCValue(4,3, 'Motor 2');
  SetRCValue(4,4, 'Motors 3');
end;
