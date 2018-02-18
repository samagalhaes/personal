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
    Pxx, Pxy, Pxo, Pyx, Pyy, Pyo, Pox, Poy, Poo: double;
	  d_at:double;
    delta_theta:double;
    z1, z2: double;


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
    if ((sensorFront < 0) OR (sensorBack < 0)) then
        marco := 0;
end;

procedure ObservationModule();
var
  L, Lw: double;
begin

  L := 0.1;  // sensors distance
  Lw := 0.5 + 0.2 - 0.025; //mark distance to square middle line

  ObsAng := aTan2 ((sensorBack - sensorFront), L);
  dw := ((sensorFront + sensorBack)/2.0)*Cos(ObsAng);

  SetRCValue(2,6, format('%.3g', [ObsAng]));
  SetRCValue(3,6, format('%.3g', [dw]));

  if ((marco = 1)) then begin
    z1 :=  Lw - dw;
    z2 := -pi()/2.0 + ObsAng;
  end;

  if ((marco = 2)) then begin
    z1 :=  -(Lw - dw);
    z2 := -pi() + ObsAng;
  end;

  if ((marco = 3)) then begin
    z1 :=  -(Lw - dw);
    z2 := pi()/2.0 + ObsAng;
  end;

  if (marco = 4) then begin
    z1 :=  Lw - dw;
    z2 := ObsAng;
  end;

  if theta > (Pi()) then     //adjust angle between -pi and pi;
    theta := theta - 2*Pi();
  if theta < (-Pi()) then
    theta := theta + 2*Pi();

  SetRCValue(6,4, format('%.3g', [x_atual]));
  SetRCValue(7,4, format('%.3g', [y_atual]));
  SetRCValue(8,4, format('%.3g', [theta*180/Pi()]));

end;

procedure Odometria(odo2, odo1:double);
begin
  d_at := (Odo1 + Odo2)/2*0.0005;
  delta_theta := (Odo1 - Odo2)*0.0005/0.1;

  if marco = 0 then begin
    theta := theta + delta_theta;

    if theta > (Pi()) then
      theta := theta - 2*Pi();

    if theta < (-Pi()) then
      theta := theta + 2*Pi();
  end;
  
  if ((marco <> 1) OR (marco <> 3)) then
    x_atual := x_atual + d_at*cos(theta + delta_theta/2);
    
  if ((marco <> 2) OR (marco <> 4)) then
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
  
  SetRCValue(1, 7, format('%.3g', [d]))
end;

procedure EstimationModule(odo2, odo1: double);
var
  d1, d2: double;
  Codo1, Codo2: double;
  d, deltaTheta: double;
  Pn_xx, Pn_xy, Pn_xo, Pn_yx, Pn_yy, Pn_yo, Pn_ox, Pn_oy, Pn_oo: double;
  p11, p12, p13, p21, p22, p23, p31, p32, p33: double;
  Q11, Q22, R11, R22: double;
  dfx13, dfx23, dfq11, dfq12, dfq21, dfq22: double;
  h11, h12, h21, h22: double;
  a, b, dObsAng_dds1, signal, L: double;
  W11, W12, W21, W22, W31, W32: double;
  v1, v2: double;
begin
  L := 0.1;

  d := d_at;
  deltaTheta := delta_theta;

  R11 := 0.002*0.002;
  R22 := R11;
  Q11 := 100*d;
  Q22 := 1000;  //Tenho de ver melhor quais os valores a por aqui

  if (marco = 0) then begin
  
    // Calculo dos parametros auxiliares
    dfx13 := -d * sin(theta + deltaTheta/2);
    dfx23 := d * cos(theta + deltaTheta/2);
    
    dfq11 := cos(theta + deltaTheta/2);
    dfq12 := -(1.0/2.0)*d * sin(theta + deltaTheta/2);
    dfq21 := sin(theta + deltaTheta/2);
    dfq22 := (1.0/2.0)*d * cos(theta + deltaTheta/2);

    // Cálculo de P
    Pn_xx := Q11*dfq11*dfq11 + Q22*dfq12*dfq12 + Pxx + Pxo*dfx13 + dfx13*(Pox + Poo*dfx13);
    Pn_yx := Pxy + Pxo*dfx23 + dfx13*(Poy + Poo*dfx23) + Q11*dfq11*dfq21 + Q22*dfq12*dfq22;
    Pn_ox := Pxo + Poo*dfx13 + Q22*dfq12;
    
    Pn_xy := Pyx + Pyo*dfx13 + dfx23*(Pox + Poo*dfx13) + Q11*dfq11*dfq21 + Q22*dfq12*dfq22;
    Pn_yy := Q11*dfq21*dfq21 + Q22*dfq22*dfq22 + Pyy + Pyo*dfx23 + dfx23*(Poy + Poo*dfx23);
    Pn_oy := Pyo + Poo*dfx23 + Q22*dfq22;
    
    Pn_xo := Pox + Poo*dfx13 + Q22*dfq12;
    Pn_yo := Poy + Poo*dfx23 + Q22*dfq22;
    Pn_oo := Poo + Q22;
    
    // Actualização de P
    Pxx := Pn_xx;
    Pxy := Pn_xy;
    Pxo := Pn_xo;

    Pyy := Pn_yy;
    Pyx := Pn_yx;
    Pyo := Pn_yo;

    Poo := Pn_oo;
    Pox := Pn_ox;
    Poy := Pn_oy;
  end;

  if ((marco = 1) or (marco = 3)) AND (dw > 0) then begin  {Confirmar este números de marco}

    // Copia de P para os p's usados
    p11 := Pxx;
    p12 := Pxy;
    p13 := Pxo;

    p21 := Pyx;
    p22 := Pyy;
    p23 := Pyo;

    P31 := Pox;
    p32 := Poy;
    p33 := Poo;

    // Cálculo de parametros auxiliares
    dObsAng_dds1 := L/(L*L + (sensorBack - sensorFront)*(sensorBack - sensorFront));
    a := cos(ObsAng)/2.0;
    b := -(sensorFront + sensorBack)*sin(ObsAng)*dObsAng_dds1/2.0;

    if (marco = 1) then signal := 1
    else if (marco = 3) then signal := -1;

    // Cálculo da matriz Delta h_r
    h11 := signal*(a+b);
    h12 := signal*(a-b);
    h21 := dObsAng_dds1;
    h22 := -dObsAng_dds1;

    // Cálculo do ganho do filtro de Kalman
    W11 := (p11*r11*h21*h21 - h11*p13*r11*h21 + p11*r22*h22*h22 - h12*p13*r22*h22 + p11*p33 - p13*p31)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    W12 := (p13*r11*h11*h11 - h21*p11*r11*h11 + p13*r22*h12*h12 - h22*p11*r22*h12)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    W21 := (p21*r11*h21*h21 - h11*p23*r11*h21 + p21*r22*h22*h22 - h12*p23*r22*h22 + p21*p33 - p23*p31)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    W22 := (p23*r11*h11*h11 - h21*p21*r11*h11 + p23*r22*h12*h12 - h22*p21*r22*h12 + p11*p23 - p13*p21)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    W31 := (p31*r11*h21*h21 - h11*p33*r11*h21 + p31*r22*h22*h22 - h12*p33*r22*h22)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    W32 := (p33*r11*h11*h11 - h21*p31*r11*h11 + p33*r22*h12*h12 - h22*p31*r22*h12 + p11*p33 - p13*p31)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    
    // Cálculo de V
    v1 := -x_atual + z1;
    v2 := -theta + z2;

    // Correção de x e theta
    x_atual := x_atual + w11*v1 + w12*v2;
    y_atual := y_atual + w21*v1 + w22*v2;
    theta := theta + w31*v1 + w32*v2;

    // Cálculo de P
    Pn_xx := (h11*h11*p11*p33*r11 - h11*h11*p13*p31*r11 + h12*h12*p11*p33*r22 - h12*h12*p13*p31*r22 + h11*h11*h22*h22*p11*r11*r22 + h12*h12*h21*h21*p11*r11*r22 - 2*h11*h12*h21*h22*p11*r11*r22)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    Pn_yx := (h11*h11*p12*p33*r11 - h11*h11*p13*p32*r11 + h12*h12*p12*p33*r22 - h12*h12*p13*p32*r22 + h11*h11*h22*h22*p12*r11*r22 + h12*h12*h21*h21*p12*r11*r22 + h11*h21*p11*p32*r11 - h11*h21*p12*p31*r11 + h12*h22*p11*p32*r22 - h12*h22*p12*p31*r22 - 2*h11*h12*h21*h22*p12*r11*r22)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    Pn_ox := (h11*h11*h22*h22*p13*r11*r22 + h12*h12*h21*h21*p13*r11*r22 + h11*h21*p11*p33*r11 - h11*h21*p13*p31*r11 + h12*h22*p11*p33*r22 - h12*h22*p13*p31*r22 - 2*h11*h12*h21*h22*p13*r11*r22)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    
    Pn_xy := (h11*h11*p21*p33*r11 - h11*h11*p23*p31*r11 + h12*h12*p21*p33*r22 - h12*h12*p23*p31*r22 + h11*h11*h22*h22*p21*r11*r22 + h12*h12*h21*h21*p21*r11*r22 + h11*h21*p11*p23*r11 - h11*h21*p13*p21*r11 + h12*h22*p11*p23*r22 - h12*h22*p13*p21*r22 - 2*h11*h12*h21*h22*p21*r11*r22)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    Pn_yy := (p11*p22*p33 - p11*p23*p32 - p12*p21*p33 + p12*p23*p31 + p13*p21*p32 - p13*p22*p31 + h21*h21*p11*p22*r11 - h21*h21*p12*p21*r11 + h11*h11*p22*p33*r11 - h11*h11*p23*p32*r11 + h22*h22*p11*p22*r22 - h22*h22*p12*p21*r22 + h12*h12*p22*p33*r22 - h12*h12*p23*p32*r22 + h11*h11*h22*h22*p22*r11*r22 + h12*h12*h21*h21*p22*r11*r22 + h11*h21*p12*p23*r11 - h11*h21*p13*p22*r11 + h12*h22*p12*p23*r22 - h12*h22*p13*p22*r22 + h11*h21*p21*p32*r11 - h11*h21*p22*p31*r11 + h12*h22*p21*p32*r22 - h12*h22*p22*p31*r22 - 2*h11*h12*h21*h22*p22*r11*r22)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    Pn_oy := (h21*h21*p11*p23*r11 - h21*h21*p13*p21*r11 + h22*h22*p11*p23*r22 - h22*h22*p13*p21*r22 + h11*h11*h22*h22*p23*r11*r22 + h12*h12*h21*h21*p23*r11*r22 + h11*h21*p21*p33*r11 - h11*h21*p23*p31*r11 + h12*h22*p21*p33*r22 - h12*h22*p23*p31*r22 - 2*h11*h12*h21*h22*p23*r11*r22)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    
    Pn_xo := (h11*h11*h22*h22*p31*r11*r22 + h12*h12*h21*h21*p31*r11*r22 + h11*h21*p11*p33*r11 - h11*h21*p13*p31*r11 + h12*h22*p11*p33*r22 - h12*h22*p13*p31*r22 - 2*h11*h12*h21*h22*p31*r11*r22)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    Pn_yo := (h21*h21*p11*p32*r11 - h21*h21*p12*p31*r11 + h22*h22*p11*p32*r22 - h22*h22*p12*p31*r22 + h11*h11*h22*h22*p32*r11*r22 + h12*h12*h21*h21*p32*r11*r22 + h11*h21*p12*p33*r11 - h11*h21*p13*p32*r11 + h12*h22*p12*p33*r22 - h12*h22*p13*p32*r22 - 2*h11*h12*h21*h22*p32*r11*r22)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    Pn_oo := (h21*h21*p11*p33*r11 - h21*h21*p13*p31*r11 + h22*h22*p11*p33*r22 - h22*h22*p13*p31*r22 + h11*h11*h22*h22*p33*r11*r22 + h12*h12*h21*h21*p33*r11*r22 - 2*h11*h12*h21*h22*p33*r11*r22)/(p11*p33 - p13*p31 + h21*h21*p11*r11 + h11*h11*p33*r11 + h22*h22*p11*r22 + h12*h12*p33*r22 - h11*h21*p13*r11 - h12*h22*p13*r22 - h11*h21*p31*r11 - h12*h22*p31*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    
    // Actualização de P
    Pxx := Pn_xx;
    Pxy := Pn_xy;
    Pxo := Pn_xo;

    Pyy := Pn_yy;
    Pyx := Pn_yx;
    Pyo := Pn_yo;

    Poo := Pn_oo;
    Pox := Pn_ox;
    Poy := Pn_oy;
  end;

  if ((marco = 2) or (marco = 4)) AND (dw > 0) then begin  {Confirmar este números de marco}

    // Copia de P para os p's usados
    p11 := Pxx;
    p12 := Pxy;
    p13 := Pxo;

    p21 := Pyx;
    p22 := Pyy;
    p23 := Pyo;

    P31 := Pox;
    p32 := Poy;
    p33 := Poo;

    // Cálculo de parametros auxiliares
    dObsAng_dds1 := L/(L*L + ((sensorBack - sensorFront))*((sensorBack - sensorFront)));
    a := Cos(ObsAng)/2;
    b := -(sensorBack + sensorFront)*(1.0/2.0)*Sin(ObsAng)*dObsAng_dds1;

    if (marco = 2) then signal := -1
    else if (marco = 4) then signal := 1;

    // Cálculo da matriz Delta h_r
    h11 := signal * (a+b);
    h12 := signal * (a-b);
    h21 := dObsAng_dds1;
    h22 := -dObsAng_dds1;

    // Cálculo do ganho do filtro de Kalman
    W11 := (p12*r11*h21*h21- h11*p13*r11*h21 + p12*r22*h22*h22 - h12*p13*r22*h22 + p12*p33 - p13*p32)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    W12 := -(- p13*r11*h11*h11 + h21*p12*r11*h11 - p13*r22*h12*h12 + h22*p12*r22*h12 + p12*p23 - p13*p22)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    W21 := (p22*r11*h21*h21 - h11*p23*r11*h21 + p22*r22*h22*h22 - h12*p23*r22*h22 + p22*p33 - p23*p32)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    W22 := (p23*r11*h11*h11 - h21*p22*r11*h11 + p23*r22*h12*h12 - h22*p22*r22*h12)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    W31 := (p32*r11*h21*h21 - h11*p33*r11*h21 + p32*r22*h22*h22 - h12*p33*r22*h22)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    W32 := (p33*r11*h11*h11 - h21*p32*r11*h11 + p33*r22*h12*h12 - h22*p32*r22*h12 + p22*p33 - p23*p32)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);

    // Cálculo de V
    v1 := -y_atual + z1;
    v2 := -theta + z2;

    // Correção de x e theta
    x_atual := x_atual + w11*v1 + w12*v2;
    y_atual := y_atual + w21*v1 + w22*v2;
    theta := theta + w31*v1 + w32*v2;

    // Cálculo de P
    Pn_xx := (p11*p22*p33 - p11*p23*p32 - p12*p21*p33 + p12*p23*p31 + p13*p21*p32 - p13*p22*p31 + h21*h21*p11*p22*r11 - h21*h21*p12*p21*r11 + h11*h11*p11*p33*r11 - h11*h11*p13*p31*r11 + h22*h22*p11*p22*r22 - h22*h22*p12*p21*r22 + h12*h12*p11*p33*r22 - h12*h12*p13*p31*r22 + h11*h11*h22*h22*p11*r11*r22 + h12*h12*h21*h21*p11*r11*r22 - h11*h21*p11*p23*r11 + h11*h21*p13*p21*r11 - h11*h21*p11*p32*r11 + h11*h21*p12*p31*r11 - h12*h22*p11*p23*r22 + h12*h22*p13*p21*r22 - h12*h22*p11*p32*r22 + h12*h22*p12*p31*r22 - 2*h11*h12*h21*h22*p11*r11*r22)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    Pn_yx := (h11*h11*p21*p33*r11 - h11*h11*p23*p31*r11 + h12*h12*p21*p33*r22 - h12*h12*p23*p31*r22 + h11*h11*h22*h22*p21*r11*r22 + h12*h12*h21*h21*p21*r11*r22 - h11*h21*p21*p32*r11 + h11*h21*p22*p31*r11 - h12*h22*p21*p32*r22 + h12*h22*p22*p31*r22 - 2*h11*h12*h21*h22*p21*r11*r22)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    Pn_ox := (h21*h21*p22*p31*r11 - h21*h21*p21*p32*r11 - h22*h22*p21*p32*r22 + h22*h22*p22*p31*r22 + h11*h11*h22*h22*p31*r11*r22 + h12*h12*h21*h21*p31*r11*r22 + h11*h21*p21*p33*r11 - h11*h21*p23*p31*r11 + h12*h22*p21*p33*r22 - h12*h22*p23*p31*r22 - 2*h11*h12*h21*h22*p31*r11*r22)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);

    Pn_xy := (h11*h11*p12*p33*r11 - h11*h11*p13*p32*r11 + h12*h12*p12*p33*r22 - h12*h12*p13*p32*r22 + h11*h11*h22*h22*p12*r11*r22 + h12*h12*h21*h21*p12*r11*r22 - h11*h21*p12*p23*r11 + h11*h21*p13*p22*r11 - h12*h22*p12*p23*r22 + h12*h22*p13*p22*r22 - 2*h11*h12*h21*h22*p12*r11*r22)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    Pn_yy := (h11*h11*p22*p33*r11 - h11*h11*p23*p32*r11 + h12*h12*p22*p33*r22 - h12*h12*p23*p32*r22 + h11*h11*h22*h22*p22*r11*r22 + h12*h12*h21*h21*p22*r11*r22 - 2*h11*h12*h21*h22*p22*r11*r22)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    Pn_oy := (h11*h11*h22*h22*p32*r11*r22 + h12*h12*h21*h21*p32*r11*r22 + h11*h21*p22*p33*r11 - h11*h21*p23*p32*r11 + h12*h22*p22*p33*r22 - h12*h22*p23*p32*r22 - 2*h11*h12*h21*h22*p32*r11*r22)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);

    Pn_xo := (h21*h21*p13*p22*r11 - h21*h21*p12*p23*r11 - h22*h22*p12*p23*r22 + h22*h22*p13*p22*r22 + h11*h11*h22*h22*p13*r11*r22 + h12*h12*h21*h21*p13*r11*r22 + h11*h21*p12*p33*r11 - h11*h21*p13*p32*r11 + h12*h22*p12*p33*r22 - h12*h22*p13*p32*r22 - 2*h11*h12*h21*h22*p13*r11*r22)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    Pn_yo := (h11*h11*h22*h22*p23*r11*r22 + h12*h12*h21*h21*p23*r11*r22 + h11*h21*p22*p33*r11 - h11*h21*p23*p32*r11 + h12*h22*p22*p33*r22 - h12*h22*p23*p32*r22 - 2*h11*h12*h21*h22*p23*r11*r22)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);
    Pn_oo := (h21*h21*p22*p33*r11 - h21*h21*p23*p32*r11 + h22*h22*p22*p33*r22 - h22*h22*p23*p32*r22 + h11*h11*h22*h22*p33*r11*r22 + h12*h12*h21*h21*p33*r11*r22 - 2*h11*h12*h21*h22*p33*r11*r22)/(p22*p33 - p23*p32 + h21*h21*p22*r11 + h11*h11*p33*r11 + h22*h22*p22*r22 + h12*h12*p33*r22 - h11*h21*p23*r11 - h11*h21*p32*r11 - h12*h22*p23*r22 - h12*h22*p32*r22 + h11*h11*h22*h22*r11*r22 + h12*h12*h21*h21*r11*r22 - 2*h11*h12*h21*h22*r11*r22);

    // Actualização de P
    Pxx := Pn_xx;
    Pxy := Pn_xy;
    Pxo := Pn_xo;

    Pyy := Pn_yy;
    Pyx := Pn_yx;
    Pyo := Pn_yo;

    Poo := Pn_oo;
    Pox := Pn_ox;
    Poy := Pn_oy;

  end;

  SetRCValue(15,2, format('%.3g', [W11]));
  SetRCValue(15,3, format('%.3g', [W12]));
  SetRCValue(16,2, format('%.3g', [W21]));
  SetRCValue(16,3, format('%.3g', [W22]));
  SetRCValue(17,2, format('%.3g', [W31]));
  SetRCValue(17,3, format('%.3g', [W32]));
  
  SetRCValue(6,8, format('%.3g', [z1]));
  SetRCValue(7,8, format('%.3g', [z2]));
  SetRCValue(6,9, format('%.3g', [v1]));
  SetRCValue(7,9, format('%.3g', [v2]));

  SetRCValue(20,2, format('%.3g', [Pxx]));
  SetRCValue(20,3, format('%.3g', [Pxy]));
  SetRCValue(20,4, format('%.3g', [Pxo]));
  SetRCValue(21,2, format('%.3g', [Pyx]));
  SetRCValue(21,3, format('%.3g', [Pyy]));
  SetRCValue(21,4, format('%.3g', [Pyo]));
  SetRCValue(22,2, format('%.3g', [Pox]));
  SetRCValue(22,3, format('%.3g', [Poy]));
  SetRCValue(22,4, format('%.3g', [Poo]));

end;

procedure ControladorB();
var
  N, Vector_A, Vector_P, Vector_F: MATRIX;
  a: double;
  VNOM, Ka, Kd: integer;
  v, w, v1, v2: double;
begin
  VNOM := 10;
  Ka := 150;
  Kd := -500;
  
  if marco <> 0 then begin
    VNOM := 2;
    Ka := 75;
    Kd := -100;
  end;

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
        if (y_atual < -0.3) then begin
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
  SensorFront := GetSensorVal(0,0)+ noiseFront;
  SensorBack := GetSensorVal(0,1); // + noiseBack;


  //GetAxisOdo(RobotIndex, AxisIndex);
  odo1 := GetAxisOdo(0,0);
  odo2 := GetAxisOdo(0,1);
  
  Odometria(odo1, odo2);
  AssociationModule();
  ObservationModule();
  EstimationModule(odo1, odo2);

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
{  Pxx := 0.5*0.5;
  Pxy := 0; 
  Pxo := 0; 
  Pyx := 0;
  Pyy := 0.5*0.5; 
  Pyo := 0; 
  Pox := 0; 
  Poy := 0; 
  Poo := (pi()/2)*(pi()/2);
}

  Pxx := 40;
  Pxy := 40;
  Pxo := 40;
  Pyx := 40;
  Pyy := 40;
  Pyo := 40;
  Pox := 40;
  Poy := 40;
  Poo := 40;

  d_at := 0;
  delta_theta := 0;

  t := 0;
  w := 1;

  ControlMode := 'keys';

  theta := -Pi()/2;
  x_atual := 0.5;
  y_atual := 0.5;
  
  d := 0;
  ObsAng := -1;
  dw := -1;

  SetRCValue(6,4, format('%.3g', [x_atual]));
  SetRCValue(7,4, format('%.3g', [y_atual]));
  SetRCValue(8,4, format('%.3g', [theta*180/Pi()]));

  state := 1;
  state_path := 1;
  marco := 0;
end;
