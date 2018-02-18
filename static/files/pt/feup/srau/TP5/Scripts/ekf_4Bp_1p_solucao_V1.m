% Codigo para Filtro Kalman Extendido
% Robôs futebol robótico com câmara móvel no topo

clear all;
N=300;
p_count=0;

% estado inicial do robô
x=[2 -2 0]';

% sinais de controlo do robô
% velocidade linear
v_t=randn(1,N)+1;
% velocidade de rotação
omega_t=randn(1,N)*0.5+0.5;
% vetor com velocidades de referência para o robô
u_t=[ v_t ; omega_t ];

% covariancias
P=eye(3)*1e-3;
% ajustar este valores de modo a aobter-se a melhor relação enter
% velocidade de convergência e sensibilidade ao ruído das medidas
Q=[0.7^2 0
    0    0.01^2];

% desvio padrão do erro das medidas dos postes
sdv_dist_1m=0.05;
sdv_ang=0.01;

% variáveis para registo da simulação
x_t=[];
x_e_t=[];

% localização dos postes
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
  % registo da evolução dos estados  
  x_t=[x_t x];
  x_e_t=[x_e_t x_e];
    
  % Simulação do robô
  v=u_t(1,i);
  omega=u_t(2,i);
  TSPAN=[0,dt];
  [T,X]=ode45('robot_5dpo',TSPAN,x,[],[v omega]);
  x=X(size(X,1),:)';
  xr=x(1); 
  yr=x(2); 
  theta_r=x(3);
 
  %propagação da estimativa do estado do robô X(k+1) = f(X(k),U)
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
  
  % calculo de grad_f_Q=df/dQ
  grad_f_Q=[cos(theta_r_e+omega*dt/2)*dt -v*dt*0.5*sin(theta_r_e+omega*dt/2); 
      sin(theta_r_e+omega*dt/2)*dt v*dt*0.5*cos(theta_r_e+omega*dt/2); 
      0 1];
  
  
  % propagação da covariancia 
  P = grad_f_X * P * grad_f_X' + grad_f_Q*Q*grad_f_Q';

          
    % simulação das medidas observadas
    distp=sqrt((xp1-xr)^2+(yp1-yr)^2);
    distp_medido=distp+randn(1)*sdv_dist_1m*distp;
    %    theta_p=atan2(yp1-yr,xp1-xr)-theta_r;
    %    theta_p_medido = theta_p + randn(1)*sdv_ang;
    z=[distp_medido];

    % valor esperado para as medidas tendo em conta o estado do robô z=h(X)
    distp_e= sqrt((xp1-xr_e)^2+(yp1-yr_e)^2);
    theta_p_e=atan2(yp1-yr_e,xp1-xr_e)-theta_r_e;
    z_e=[distp_e];

    % matriz de covariancia do ruído das medidas
    R= [ (distp_e*sdv_dist_1m)^2 ];

    % calculo de dH/dX
    grad_h_X  = [-(xp1-xr_e)/distp_e, -(yp1-yr_e)/distp_e, 0]; 

    % Kalman Gain
    k = P * grad_h_X' * inv(grad_h_X * P * grad_h_X' + R);

  % Actualização da covariencia
  P=(eye(3)-k * grad_h_X) * P;
  
  % Actualiz estado estimado
  x_e = x_e + k * (z - z_e);
    
    %propagação da estimativa do estado do robô X(k+1) = f(X(k),U)
  xr_e=x_e(1); 
  yr_e=x_e(2); 
  theta_r_e=x_e(3);  
  
    
    
    % simulação das medidas observadas
    distp=sqrt((xp2-xr)^2+(yp2-yr)^2);
    distp_medido=distp+randn(1)*sdv_dist_1m*distp;
%    theta_p=atan2(yp2-yr,xp2-xr)-theta_r;
%    theta_p_medido = theta_p + randn(1)*sdv_ang;
    z=[distp_medido];
    
    % valor esperado para as medidas tendo em conta o estado do robô z=h(X)
    distp_e= sqrt((xp2-xr_e)^2+(yp2-yr_e)^2);
    theta_p_e=atan2(yp2-yr_e,xp2-xr_e)-theta_r_e;
    z_e=[distp_e];
    
    % matriz de covariancia do ruído das medidas
    R= [ (distp_e*sdv_dist_1m)^2 ];
    
    % calculo de dH/dX
    grad_h_X  = [-(xp2-xr_e)/distp_e, -(yp2-yr_e)/distp_e, 0];
    
        % Kalman Gain
    k = P * grad_h_X' * inv(grad_h_X * P * grad_h_X' + R);

  % Actualização da covariencia
  P=(eye(3)-k * grad_h_X) * P;
  
  % Actualiz estado estimado
  x_e = x_e + k * (z - z_e);
    
    %propagação da estimativa do estado do robô X(k+1) = f(X(k),U)
  xr_e=x_e(1); 
  yr_e=x_e(2); 
  theta_r_e=x_e(3); 
  
  
    
    
    % simulação das medidas observadas
    distp=sqrt((xp3-xr)^2+(yp3-yr)^2);
    distp_medido=distp+randn(1)*sdv_dist_1m*distp;
%    theta_p=atan2(yp2-yr,xp2-xr)-theta_r;
%    theta_p_medido = theta_p + randn(1)*sdv_ang;
    z=[distp_medido];
    
    % valor esperado para as medidas tendo em conta o estado do robô z=h(X)
    distp_e= sqrt((xp3-xr_e)^2+(yp3-yr_e)^2);
    theta_p_e=atan2(yp3-yr_e,xp3-xr_e)-theta_r_e;
    z_e=[distp_e];
    
    % matriz de covariancia do ruído das medidas
    R= [ (distp_e*sdv_dist_1m)^2 ];
    
    % calculo de dH/dX
    grad_h_X  = [-(xp3-xr_e)/distp_e, -(yp3-yr_e)/distp_e, 0];
    
        % Kalman Gain
    k = P * grad_h_X' * inv(grad_h_X * P * grad_h_X' + R);

  % Actualização da covariencia
  P=(eye(3)-k * grad_h_X) * P;
  
  % Actualiz estado estimado
  x_e = x_e + k * (z - z_e);
    
    %propagação da estimativa do estado do robô X(k+1) = f(X(k),U)
  xr_e=x_e(1); 
  yr_e=x_e(2); 
  theta_r_e=x_e(3);
    
 
    % simulação das medidas observadas
    distp=sqrt((xp4-xr)^2+(yp4-yr)^2);
    distp_medido=distp+randn(1)*sdv_dist_1m*distp;
%    theta_p=atan2(yp2-yr,xp2-xr)-theta_r;
%    theta_p_medido = theta_p + randn(1)*sdv_ang;
    z=[distp_medido];
    
    % valor esperado para as medidas tendo em conta o estado do robô z=h(X)
    distp_e= sqrt((xp4-xr_e)^2+(yp4-yr_e)^2);
    theta_p_e=atan2(yp4-yr_e,xp4-xr_e)-theta_r_e;
    z_e=[distp_e];
    
    % matriz de covariancia do ruído das medidas
    R= [ (distp_e*sdv_dist_1m)^2 ];
    
    % calculo de dH/dX
    grad_h_X  = [-(xp4-xr_e)/distp_e, -(yp4-yr_e)/distp_e, 0];
    
        % Kalman Gain
    k = P * grad_h_X' * inv(grad_h_X * P * grad_h_X' + R);

  % Actualização da covariencia
  P=(eye(3)-k * grad_h_X) * P;
  
  % Actualiz estado estimado
  x_e = x_e + k * (z - z_e);
 
end

% visualização dos resultados
L=0.01;
plot (  x_t(1,:),   x_t(2,:) ,'o',  ...
        x_t(1,:)+L*cos(x_t(3,:)),   x_t(2,:)+L*sin(x_t(3,:)) ,'.'   , ...
        x_e_t(1,:),   x_e_t(2,:) ,'o',  ...
        x_e_t(1,:)+L*cos(x_e_t(3,:)),   x_e_t(2,:)+L*sin(x_e_t(3,:)) ,'.' ,  ...
        xp1,yp1 ,'p', xp2,yp2, 'h' ...
        ),
legend(   'xy',          'dir', 'xy_e',          'dir_e', 'Pole 1',  'Pole 2' ), 


         
