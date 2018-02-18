function F = robot_5dpo(t,y,FLAG,u)
% SIMULA��O DE UM ROBOT MOVEL
% 
% Vector estado:   X(t)=[deslocamento segundo o eixo x - x(t)
%                        deslocamento segundo o eixo y - y(t)
%                        posi��o �ngular - a(t)]
%
%                  u(t)=[velocidade linear - v(t)
%                        velocidade �ngular - w(t)]
%
%
% Equa��es: dx(t)/dt = v(t)*cos(a(t))
%           dy(t)/dt = v(t)*sin(a(t))
%           da(t)/dt = w(t)


% Retorna dy/dt = f(t,y).
F = [u(1)*cos(y(3))
     u(1)*sin(y(3))
     u(2)];
  
