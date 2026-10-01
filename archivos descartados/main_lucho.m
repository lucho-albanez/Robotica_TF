clc; clear; close all;

% 1. Cargamos el robot en memoria
robot;

%% --- PREPARACIÓN DE LOS PARÁMETROS DE ENTRADA ---

% a) q0: semilla inicial
q0 = [0, 0, 0, 0, 0, 0];

% b) sucesión de poses
% creamos una trayectoria articular y obtenemos sus matrices cartesianas.
% Vamos desde la posición cero hasta una posición determinada.
q_inicio = [0, 0, 0, 0, 0, 0];
q_fin = [0, pi/4, pi/4, 0, 0, 0];
trayectoria_real = jtraj(q_inicio, q_fin, 30); % 30 puntos
plot(1:30,trayectoria_real);

%% --- CINEMATICA DIRECTA ---
% Obtenemos las poses cartesianas (T será de tamaño 4x4x30)
T = R.fkine(trayectoria_real);

%% --- PRUEBA 1: q_mejor = true (Única solución) ---
disp('--- PROBANDO q_mejor = true ---');
q_mejor = true;

[exito_1, q_resultado_1, mensaje_1] = IK_numeric_lucho(R, T, q_mejor, q0);

disp(['Éxito: ', num2str(exito_1)]);
disp(['Mensaje: ', mensaje_1]);

% Si tuvo éxito, graficamos la solución
if exito_1
    figure('Name', 'Prueba Única Solución');
    R.plot(q_resultado_1, 'workspace', workspace, 'scale', 0.5, 'notiles');
end

%% --- PRUEBA 2: q_mejor = false (Múltiples soluciones) ---
disp('--- PROBANDO q_mejor = false ---');
q_mejor = false;

[exito_2, q_resultado_2, mensaje_2] = IK_numeric_lucho(R, T, q_mejor, q0);

disp(['Éxito: ', num2str(exito_2)]);
disp(['Mensaje: ', mensaje_2]);

% Si tuvo éxito, q_resultado_2 será un Cell Array con varias trayectorias.
% Podemos ver cuántas encontró y graficar la segunda (si encontró más de una).
if exito_2
    num_soluciones = length(q_resultado_2);
    disp(['¡Se encontraron ', num2str(num_soluciones), ' soluciones distintas!']);
    for i = 1:num_soluciones
        % Abre una ventana gráfica independiente numerada para cada solución
        figure('Name', ['Configuración #', num2str(i)]);

        % Animar la trayectoria completa
        R.plot(q_resultado_2{i}, 'workspace', workspace, 'scale', 0.5, 'notiles');
    end
end


