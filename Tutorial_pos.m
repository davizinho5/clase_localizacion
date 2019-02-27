% -------------------------------------------------------------------------
% Filtro de Kalman para la localización de un robot móvil
% El modelo de movimiento del robot es:
%
% posicion_actual = posicion_anterior + velocidad*tiempo transcurrido
% 
% El sensor del robot es capaz de medir la velocidad actual
% -------------------------------------------------------------------------

close all
clear all
  
% Numero de iteraciones
Niterations=100;
% Interval temporal entre iteracion
dt = 0.1;
% Variable temporal para las graficas
t=0:dt:dt*Niterations;

% Sistema con velocidad constante
V = 0.2; % [m/s]

% Vector de estados iniciales (posicion y velocidad)
xEst = [0.0; V];

% Vectores de estados para comparar resultados
xTrue = xEst;
x_medida = xEst;
x_filtrada = xEst;

% Vector de medidas
z = V;
Z = z';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Ecuacion de Movimiento %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ecuacion de Movimiento: relaciona el vector de estado con estado anterior
% y entrada
% Xk = Xk_prev + Vel*dt + Noise
% Vk = Xk_prev + Noise
A = [1 dt 
     0 1];

% No hay entrada en este sistema
B = [];

% Q matriz de covarinza del ruido de la ecuacion de movimiento (porque se producen errores al moverse)
sigma_model = 0.2; % desviación típica de la gausiana
Q = [sigma_model  0  ;
               0  sigma_model].^2;
  
% Ecuacion de medicion: relaciona el vector de estado con la medicion actual
% NO se mide posicion, SI se mide velocidad
C = [0 1];

% R es la covarianza del ruido en las mediciones
sigma_meas = 0.05; % m/sec
R = sigma_meas^2;

% Matriz P, de confianza
% Contiene la verosimilitud de la estimacion
P = eye(2);         

% Ruido para la simulación de los y sensores reales
sim_mov = []; % NO SE UTILIZA PORQUE NO HAY ENTRADA AL SISTEMA
sim_sen = [0.5].^2;

% Main loop
for k=1:Niterations 
    
    %% SIMULACION DE ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % vector de entradas al sistema (NO HAY)
    u = [];
     
    % Simular medida de los sensores
    z = C*(xTrue(:,k)) + sim_sen*randn(1,1);
    Z = [Z; z'];
    % SIMULACION DE ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
    %% --------- KF ----------- 
    % Prediccion
    xPred= A*xEst; 
    PPred= A*P*A' + Q;    
      
    % Update
    K = PPred*C'*inv(C*PPred*C' + R);
    xEst = xPred + K*(z - C*xPred);
    P = (eye(size(xEst,1)) - K*C)*PPred; 
    % ------ KF -------- 
   
    %% SIMULACION DE ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Resultado movimiento perfecto, medido y filtrado
    xTrue = [xTrue A*xTrue(:,k)];
    x_medida = [x_medida A*[x_medida(1,k); z]];
    x_filtrada = [x_filtrada A*[x_filtrada(1,k); xEst(2)]];
    % SIMULACION DE ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Dibujar
    % Dibujar solo cada 5 iteraciones
    if rem(k,3)==0
        figure(1), subplot(1,2,1), hold on
        plot(t(1:k),xTrue(1,1:k),'b','LineWidth',2), 
        plot(t(1:k),x_medida(1,1:k),'g','LineWidth',2), 
        plot(t(k),xEst(1,1),'*k');
        plot(t(1:k),x_filtrada(1,1:k),'r','LineWidth',2);

        figure(1), subplot(1,2,2), hold on  
        plot(t(1:k),xTrue(2,1:k),'b','LineWidth',2);
        plot(t(1:k),x_medida(2,1:k),'g','LineWidth',2);
        plot(t(1:k),x_filtrada(2,1:k),'r','LineWidth',2);
    end
    % Dibujar %%
end

figure(1), subplot(1,2,1), hold on
xlabel('Tiempo (s)');
ylabel('Distancia recorrida(m)');
legend('Ground Truth', 'Sensor', 'Estimada', 'Filtrada');

figure(1), subplot(1,2,2), hold on
xlabel('Tiempo (s)');
ylabel('Velocidad del movimiento (m/s)');
legend('Velocidad verdadera','Velocidad medida','Velocidad estimada');  
