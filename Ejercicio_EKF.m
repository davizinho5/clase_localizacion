% -------------------------------------------------------------------------
% Filtro de Extendido de Kalman para la localizaci�n de un robot m�vil
% -------------------------------------------------------------------------
 
close all;
clear all;
 
disp('Extended Kalman Filter (EKF)')
 
% Numero de iteraciones
Niterations=600;
% Interval temporal entre iteracion
dt = 0.1;
% Variable temporal para las graficas
t=0:dt:dt*Niterations;
 
% Velocidades m�ximas
Vmax = 1.0; % [m/s]
Wmax = 5;   % [deg/s]

% Definir vector de estado y sus valores iniciales
 

% Vectores de estados para comparar resultados
xTrue = xEst;
x_medida = xEst;
x_filtrada = xEst;
 
% Vector de medidas

Z = z';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Ecuacion de Movimiento %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matrices A y B

% Ruido del sistema, Q

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Observaci�n de los sensores %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matriz H

% Ruido de la obsevaci�n, R


% Matriz P, se inicializa como diagonal de 1s 
P = eye();

% Ruido para la simulaci�n del robot y sensores reales
sim_mov = 
sim_sen = 

% Main loop
for i=1 : Niterations
 
    %% SIMULACION DE ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % crear el vector de entradas al sistema 
   
    % Simular Observacion de los sensores
 
    Z = [Z; z'];
    % SIMULACION DE ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% -------- EKF ----------
    % Prediccion

    
    % Update
    
   
    % -------- EKF ----------
    
    %% SIMULACION DE ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xTrue = [xTrue  ];
    x_medida = [x_medida  ];
    x_filtrada = [x_filtrada  ];
    % SIMULACION DE ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
    %% Dibujar - DESCOMENTAR CUANDO NO TENGAIS ERRORES
% %     if rem(k,5)==0
% %         figure(1),
% %         plot(xTrue(1,1:k),xTrue(2,1:k),'.b');hold on;
% %         plot(x_medida(1,1:k),x_medida(2,1:k),'.g');hold on;
% %         plot(xEst(1),xEst(2),'.k');hold on;
% %         plot(x_filtrada(1,1:k),x_filtrada(2,1:k),'.r');hold on;
% %         % ShowErrorEllipse(xEst,P);
% %         axis equal;
% %         grid on;
% %     end
    % Dibujar
    
end
%% Dibujar - DESCOMENTAR CUANDO NO TENGAIS ERRORES
% % figure(1), hold on
% % xlabel('Tiempo (s)');
% % ylabel('Distancia recorrida(m)');
% % legend('Ground Truth', 'Sensor', 'Estimada', 'Filtrada');


%% Funciones


