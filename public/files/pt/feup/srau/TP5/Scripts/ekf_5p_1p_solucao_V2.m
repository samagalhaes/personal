% Codigo para Filtro Kalman Extendido
% Rob�s futebol rob�tico com c�mara m�vel no topo

clear all;
N=300;
p_count=0;

% estado inicial do rob�
x=[2 -2 0]';

% sinais de controlo do rob�
% velocidade linear
v_t=randn(1,N)+1;
% velocidade de rota��o
omega_t=randn(1,N)*0.5+0.5;
% vetor com velocidades de refer�ncia para o rob�
u_t=[ v_t ; omega_t ];

% covariancias
P=eye(3)*1e-3;
% ajustar este valores de modo a aobter-se a melhor rela��o enter
% velocidade de converg�ncia e sensibilidade ao ru�do das medidas
Q=[0.1^2 0
    0    0.05^2];

% desvio padr�o do erro das medidas dos postes
sdv_dist_1m=0.05;
sdv_ang=0.01;

% vari�veis para registo da simula��o
x_t=[];
x_e_t=[];

% localiza��o dos postes
xp1=5;
yp1=-2.5;
xp2=5;
yp2=2.5;
xp3=-5;
yp3=2.5;
xp4=-5;
yp4=-2.5;

% valor inicial do estado estimado
x_e=[2.5 0 0]';
xr_e=x_e(1); 
yr_e=x_e(2); 
theta_r_e=x_e(3);

% periodo de controlo
dt=0.040;

for i=1:N
  % registo da evolu��o dos estados  
  x_t=[x_t x];
  x_e_t=[x_e_t x_e];
    
  % Simula��o do rob�
  v=u_t(1,i);
  omega=u_t(2,i);
  TSPAN=[0,dt];
  [T,X]=ode45('robot_5dpo',TSPAN,x,[],[v omega]);
  x=X(size(X,1),:)';
  xr=x(1); 
  yr=x(2); 
  theta_r=x(3);
 
  %propaga��o da estimativa do estado do rob� X(k+1) = f(X(k),U)
  xr_e=x_e(1); 
  yr_e=x_e(2); 
  theta_r_e=x_e(3);  
  xr_e=xr_e+v*cos(theta_r_e+omega*dt/2)*dt; 
  yr_e=yr_e+v*sin(theta_r_e+omega*dt/2)*dt; 
  theta_r_e=theta_r_e+omega*dt;  
  x_e=[xr_e ; yr_e ; theta_r_e];
  
  % calculo de grad_f_X=df/dX 
  grad_f_X=[ 1 0 -v*dt*sin(theta_r_e+omega*dt/2);
             0 1 v*dt*cos(theta_r_e+omega*dt/2);
             0 0 1];
  
  % calculo de grad_f_U=df/dU
  grad_f_U=[cos(theta_r_e+omega*dt/2)*dt -v*dt*0.5*sin(theta_r_e+omega*dt/2); 
            sin(theta_r_e+omega*dt/2)*dt v*dt*0.5*cos(theta_r_e+omega*dt/2); 
            0 1];
  
  % propaga��o da covariancia 
  P = grad_f_X * P * grad_f_X' + grad_f_U*Q*grad_f_U';
         
  %% Visualizacao do poste 1
  % simula��o das medidas observadas
  distp=sqrt((xp1-xr)^2+(yp1-yr)^2);
  distp_medido=distp+randn(1)*sdv_dist_1m*distp;
  theta_p=atan2(yp1-yr,xp1-xr)-theta_r;
  theta_p_medido = theta_p + randn(1)*sdv_ang;
%  z=[distp_medido; theta_p_medido];
  
  stddev_xp= 0.01*distp * cos(theta_p); 
  stddev_yp= 0.01*distp * sin(theta_p);
  xp1_n = xp1 * randn(1)*stddev_xp;
  yp1_n = yp1 * randn(1)*stddev_yp;

  xpr = (xp1_n - xr)*cos(theta_r) + (yp1_n-yr)*sin(theta_r);
  ypr = (yp1_n - yr)*cos(theta_r) - (xp1_n-xr)*sin(theta_r);
  z = [xpr; ypr];
    
  % valor esperado para as medidas tendo em conta o estado do rob� z=h(X)
  distp_e= sqrt((xp1-xr_e)^2+(yp1-yr_e)^2);
  theta_p_e=atan2(yp1-yr_e,xp1-xr_e)-theta_r_e;

  xpr_e = (xp1_n - xr_e)*cos(theta_r_e) + (yp1_n-yr_e)*sin(theta_r_e);
  ypr_e = (yp1_n - yr_e)*cos(theta_r_e) - (xp1_n-xr_e)*sin(theta_r_e);
    
  z_e=[xpr_e; ypr_e];
    
  % matriz de covariancia do ru�do das medidas
%   R= [ (distp_e*sdv_dist_1m)^2 0 
%       0 sdv_ang^2 ];
  R = [stddev_xp^2      0     ; ...
          0        stddev_yp^2];
  % calculo de dH/dX
  grad_h_X = [-cos(theta_r_e) -sin(theta_r_e) ypr_e; ...
              sin(theta_r_e)  -cos(theta_r_e) -xpr_e];
          
  % Kalman Gain
  k = P * grad_h_X' * inv(grad_h_X * P * grad_h_X' + R);

  % Actualiza��o da covariencia
  P=(eye(3)-k * grad_h_X) * P;
  
  % Actualiz estado estimado
  x_e = x_e + k * (z - z_e);
  
  %propaga��o da estimativa do estado do rob� X(k+1) = f(X(k),U)
  xr_e=x_e(1); 
  yr_e=x_e(2); 
  theta_r_e=x_e(3);   
  
  %% Vizualizacao do poste 2
  % simula��o das medidas observadas
  distp=sqrt((xp2-xr)^2+(yp2-yr)^2);
  distp_medido=distp+randn(1)*sdv_dist_1m*distp;
  theta_p=atan2(yp2-yr,xp2-xr)-theta_r;
  theta_p_medido = theta_p + randn(1)*sdv_ang;
    
  stddev_xp= 0.01*distp * cos(theta_p); 
  stddev_yp= 0.01*distp * sin(theta_p);
  xp2_n = xp2 * randn(1)*stddev_xp;
  yp2_n = yp2 * randn(1)*stddev_yp;

  xpr = (xp2_n - xr)*cos(theta_r) + (yp2_n-yr)*sin(theta_r);
  ypr = (yp2_n - yr)*cos(theta_r) - (xp2_n-xr)*sin(theta_r);
  z = [xpr; ypr];  
  % valor esperado para as medidas tendo em conta o estado do rob� z=h(X)
  distp_e= sqrt((xp2-xr_e)^2+(yp2-yr_e)^2);
  theta_p_e=atan2(yp2-yr_e,xp2-xr_e)-theta_r_e;
%  z_e=[distp_e; theta_p_e];

  xpr_e = (xp2_n - xr_e)*cos(theta_r_e) + (yp2_n-yr_e)*sin(theta_r_e);
  ypr_e = (yp2_n - yr_e)*cos(theta_r_e) - (xp2_n-xr_e)*sin(theta_r_e);
  
  z_e=[xpr_e; ypr_e];
  
  % matriz de covariancia do ru�do das medidas
%   R= [ (distp_e*sdv_dist_1m)^2 0 
%       0 sdv_ang^2 ];
  R = [stddev_xp^2      0     ; ...
          0        stddev_yp^2];
  
  % calculo de dH/dX
%   grad_h_X  = [-(xp2-xr_e)/distp_e, -(yp2-yr_e)/distp_e, 0;
%                (yp2-yr_e)/(distp_e^2), -(xp2-xr_e)/(distp_e^2), -1];
  grad_h_X = [-cos(theta_r_e) -sin(theta_r_e) ypr; ...
              sin(theta_r_e)  -cos(theta_r_e) -xpr];
            
  % Kalman Gain
  k = P * grad_h_X' * inv(grad_h_X * P * grad_h_X' + R);

  % Actualiza��o da covariencia
  P=(eye(3)-k * grad_h_X) * P;
  
  % Actualiz estado estimado
  x_e = x_e + k * (z - z_e);
  
  %propaga��o da estimativa do estado do rob� X(k+1) = f(X(k),U)
  xr_e=x_e(1); 
  yr_e=x_e(2); 
  theta_r_e=x_e(3); 
             
  %% Vizualizacao do poste 3           
  % simula��o das medidas observadas
  distp=sqrt((xp3-xr)^2+(yp3-yr)^2);
  distp_medido=distp+randn(1)*sdv_dist_1m*distp;
  theta_p=atan2(yp3-yr,xp3-xr)-theta_r;
  theta_p_medido = theta_p + randn(1)*sdv_ang;
  
  stddev_xp= 0.01*distp * cos(theta_p); 
  stddev_yp= 0.01*distp * sin(theta_p);
  xp3_n = xp3 * randn(1)*stddev_xp;
  yp3_n = yp3 * randn(1)*stddev_yp;

  xpr = (xp3_n - xr)*cos(theta_r) + (yp3_n-yr)*sin(theta_r);
  ypr = (yp3_n - yr)*cos(theta_r) - (xp3_n-xr)*sin(theta_r);
  z = [xpr; ypr];

  % valor esperado para as medidas tendo em conta o estado do rob� z=h(X)
   distp_e= sqrt((xp3-xr_e)^2+(yp3-yr_e)^2);
   theta_p_e=atan2(yp3-yr_e,xp3-xr_e)-theta_r_e;
%   z_e=[distp_e; theta_p_e];

  xpr_e = (xp3_n - xr_e)*cos(theta_r_e) + (yp3_n-yr_e)*sin(theta_r_e);
  ypr_e = (yp3_n - yr_e)*cos(theta_r_e) - (xp3_n-xr_e)*sin(theta_r_e);
    
  z_e=[xpr_e; ypr_e];

  % matriz de covariancia do ru�do das medidas
%   R= [ (distp_e*sdv_dist_1m)^2 0 
%       0 sdv_ang^2 ];
  R = [stddev_xp^2      0     ; ...
          0        stddev_yp^2];

  % calculo de dH/dX
%   grad_h_X  = [-(xp3-xr_e)/distp_e, -(yp3-yr_e)/distp_e, 0;
%                (yp3-yr_e)/(distp_e^2), -(xp3-xr_e)/(distp_e^2), -1];
  grad_h_X = [-cos(theta_r_e) -sin(theta_r_e) ypr; ...
              sin(theta_r_e)  -cos(theta_r_e) -xpr];
          
  % Kalman Gain
  k = P * grad_h_X' * inv(grad_h_X * P * grad_h_X' + R);

  % Actualiza��o da covariencia
  P=(eye(3)-k * grad_h_X) * P;
  
  % Actualiz estado estimado
  x_e = x_e + k * (z - z_e);
  
  %propaga��o da estimativa do estado do rob� X(k+1) = f(X(k),U)
  xr_e=x_e(1); 
  yr_e=x_e(2); 
  theta_r_e=x_e(3);  
  
  %% Vizualizacao do poste 4
  % simula��o das medidas observadas
  distp=sqrt((xp4-xr)^2+(yp4-yr)^2);
  distp_medido=distp+randn(1)*sdv_dist_1m*distp;
  theta_p=atan2(yp4-yr,xp4-xr)-theta_r;
  theta_p_medido = theta_p + randn(1)*sdv_ang;
  
  stddev_xp= 0.01*distp * cos(theta_p); 
  stddev_yp= 0.01*distp * sin(theta_p);
  xp4_n = xp4 * randn(1)*stddev_xp;
  yp4_n = yp4 * randn(1)*stddev_yp;

  xpr = (xp4_n - xr)*cos(theta_r) + (yp4_n-yr)*sin(theta_r);
  ypr = (yp4_n - yr)*cos(theta_r) - (xp4_n-xr)*sin(theta_r);
  z = [xpr; ypr];
  
  % valor esperado para as medidas tendo em conta o estado do rob� z=h(X)
   distp_e= sqrt((xp4-xr_e)^2+(yp4-yr_e)^2);
   theta_p_e=atan2(yp4-yr_e,xp4-xr_e)-theta_r_e;
%   z_e=[distp_e; theta_p_e];

  xpr_e = (xp4_n - xr_e)*cos(theta_r_e) + (yp4_n-yr_e)*sin(theta_r_e);
  ypr_e = (yp4_n - yr_e)*cos(theta_r_e) - (xp4_n-xr_e)*sin(theta_r_e);
    
  z_e=[xpr_e; ypr_e];
   
  % matriz de covariancia do ru�do das medidas
%   R= [ (distp_e*sdv_dist_1m)^2 0 
%       0 sdv_ang^2 ];
  R = [stddev_xp^2      0     ; ...
          0        stddev_yp^2];
    
  % calculo de dH/dX
%   grad_h_X  = [-(xp4-xr_e)/distp_e, -(yp4-yr_e)/distp_e, 0;
%                (yp4-yr_e)/(distp_e^2), -(xp4-xr_e)/(distp_e^2), -1];
  grad_h_X = [-cos(theta_r_e) -sin(theta_r_e) ypr; ...
              sin(theta_r_e)  -cos(theta_r_e) -xpr];
            
  % Kalman Gain
  k = P * grad_h_X' * inv(grad_h_X * P * grad_h_X' + R);

  % Actualiza��o da covariencia
  P=(eye(3)-k * grad_h_X) * P;
  
  % Actualiz estado estimado
  x_e = x_e + k * (z - z_e);
  end;

% visualiza��o dos resultados
L=0.01;
plot (  x_t(1,:),   x_t(2,:) ,'o',  ...
        x_t(1,:)+L*cos(x_t(3,:)),   x_t(2,:)+L*sin(x_t(3,:)) ,'.'   , ...
        x_e_t(1,:),   x_e_t(2,:) ,'o',  ...
        x_e_t(1,:)+L*cos(x_e_t(3,:)),   x_e_t(2,:)+L*sin(x_e_t(3,:)) ,'.' ,  ...
        xp1,yp1 ,'p', xp2,yp2, 'h' ...
        ),
legend(   'xy',          'dir', 'xy_e',          'dir_e', 'Pole 1',  'Pole 2' ), 


         
