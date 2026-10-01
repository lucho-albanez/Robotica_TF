clc;clear;close all;

robot;

syms q1 q2 q3 q4 q5 q6 real
q = [q1, q2, q3, q4, q5, q6];

tic 

% traslado del sistema de referencia de la herramienta al centro de la muñeca
R.tool = transl(0, 0, -0.075);

J = R.jacob0(q);

% verificación. elementos de J(1:3,4:6) deben ser ceros
disp(simplify(J(1:3,4:6)));


% extraigo matriz superior izquierda, asociada al brazo - velocidades lineales
Jv = simplify(J(1:3, 1:3));

% matriz inferior derecha, asociada a la muñeca - velocidades angulares
Jw = simplify(J(4:6, 4:6));

det_arm = det(Jv);
det_wrist = det(Jw);

det_arm = simplify(det_arm);
det_wrist = simplify(det_wrist);

detJ_fact = simplify(det_arm * det_wrist);
disp(detJ_fact);

% tiempo transcurrido
toc


    