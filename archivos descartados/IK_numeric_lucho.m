function [exito, q_resultado, mensaje] = IK_numeric_lucho(R,T,q_mejor,q0)

% Inicialización de variables de salida al comienzo (buena practica)
exito = false;
q_resultado = [];
mensaje = 'Error desconocido';

% Mascara - consideramos las 6 articulaciones activas
M = [1,1,1,1,1,1];
%cantidad de iteraciones
itmax = 500;
tol = 1e-3;

%% --- q_mejor = true -> unica solucion ---
if q_mejor == true
    try
        q_temp = R.ikine(T, 'q0', q0, 'mask', M);
        % si pudo calcular la cinematica inversa, actualizamos las salidas
        exito = true;
        q_resultado = q_temp;
        mensaje = 'Solución encontrada con éxito';
    
    catch ME
        mensaje = ['Solución no encontrada', ME.message];
    end
%% --- q_mejor = false -> multiples soluciones ---        
else
    soluciones_encontradas = {};
    % generando nuevo valor semilla de q0
    q_semilla_nuevo = zeros(1,6);
    
    for j=1:itmax
        % genero semilla aleatoria a partir de los límites articulares
        for i = 1:6
        q_semilla_nuevo(i) = R.qlim(i,1) + (R.qlim(i,2)-R.qlim(i,1))*rand(1,1); % formato: a + (b-a)*rand()
        end
        try
            q_temp = R.ikine(T,'q0', q_semilla_nuevo, 'mask', M);
            if isempty(q_temp)
                continue
            end
            es_valida = true;
            for i = 1:6
                % Comparamos el mínimo y máximo de toda la columna (articulación i) contra sus límites
                if min(q_temp(:, i)) < R.qlim(i, 1) || max(q_temp(:, i)) > R.qlim(i, 2)
                    es_valida = false;
                    break; % Si una articulación falla, dejamos de revisar las demás
                end
            end
            if es_valida == false
                continue;
            end
            % bloque de verificacion de soluciones repetidas
            es_repetida = false;           
            for k = 1:length(soluciones_encontradas)
                sol_guardada = soluciones_encontradas{k};
                vect_a = q_temp(1,:);
                vect_b = sol_guardada(1,:);
                distancia = norm(vect_a - vect_b);
                if distancia < tol
                    es_repetida = true;
                    break;
                end
            end 
            if es_repetida == false
                soluciones_encontradas{end+1} = q_temp;
                exito = true;
                mensaje = 'Se encontraron múltiples soluciones';                    
            end 
                        
        catch ME
            mensaje = ['Solución no encontrada', ME.message];
        end
    end
    q_resultado = soluciones_encontradas;

end

end

