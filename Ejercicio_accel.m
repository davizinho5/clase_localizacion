% -------------------------------------------------------------------------
% Filtro de Kalman para la localización de un robot móvil
% El modelo de movimiento del robot es:
% posicion_actual = posicion_anterior + velocidad*tiempo transcurrido
% 
% El sensor del robot es capaz de medir la velocidad actual
% -------------------------------------------------------------------------

close all
clear all
  
% Numero de iteraciones
Niterations=200;
% Interval temporal entre iteracion
dt = 0.1;
% Variable temporal para las graficas
t=0:dt:dt*Niterations;

% Entrada al sistema, es fija
accel = 0.1;

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
%%% Observación de los sensores %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matriz H

% Ruido de la obsevación, R


% Matriz P, se inicializa como diagonal de 1s 
P = eye();         

% Ruido para la simulación del robot y sensores reales
sim_mov = 
sim_sen = 

% Main loop
for k=1:Niterations 
    
    %% SIMULACION DE ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % crear el vector de entradas al sistema 
   
    % Simular Observacion de los sensores
 
    Z = [Z; z'];
    % SIMULACION DE ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
    %% -------- KF ----------
    % Prediccion

    
    % Update
  

    % -------- KF ----------


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
    
%     pause;
end
%% Dibujar - DESCOMENTAR CUANDO NO TENGAIS ERRORES
% % figure(1), hold on
% % xlabel('Tiempo (s)');
% % ylabel('Distancia recorrida(m)');
% % legend('Ground Truth', 'Sensor', 'Estimada', 'Filtrada');

