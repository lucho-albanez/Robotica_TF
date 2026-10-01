%% SINGULARIDADES DEL FANUC M-10iD/8L - ANALISIS Y VISUALIZACION
% Jacobiano referido al CENTRO DE MUNECA (origen del frame 4).
%
%   det(J)  = det(Jv) * det(Jw)
%   det(Jv) = a2 * r(q2,q3) * ( d4*cos(q3) - a3*sin(q3) )
%   det(Jw) = -sin(q5)
%   r(q2,q3) = a1 - a2*sin(q2) - a3*sin(q2+q3) + d4*cos(q2+q3)
%
% Requiere: Robotics Toolbox (Peter Corke), MATLAB R2016b o superior
% (funciones locales al final del script).

robot;   % crea R, dh, workspace (y limpia el workspace)

%% 0. PARAMETROS (tomados de la matriz dh de robot.m) Y FUNCIONES ANALITICAS
a1 = dh(1,3);  d1 = dh(1,2);
a2 = dh(2,3);
a3 = dh(3,3);  d3 = dh(3,2);
d4 = dh(4,2);

qlim = R.qlim;                       % 6x2 [rad]
off  = R.offset;                     % offsets de las articulaciones
pb   = transl(R.base); pb = pb(:)';  % traslacion de la base [x y z]

r_fun   = @(q2,q3) a1 - a2*sin(q2) - a3*sin(q2+q3) + d4*cos(q2+q3);
elb_fun = @(q3) d4*cos(q3) - a3*sin(q3);
detJv   = @(q2,q3) a2 .* r_fun(q2,q3) .* elb_fun(q3);
detJw   = @(q5) -sin(q5);

% valores singulares dentro de los limites articulares
q3_codo = atan2(d4,a3) + (-2:2)*pi;
q3_codo = q3_codo(q3_codo >= qlim(3,1) & q3_codo <= qlim(3,2));
q5_mun  = (-2:2)*pi;
q5_mun  = q5_mun(q5_mun >= qlim(5,1) & q5_mun <= qlim(5,2));

% configuraciones singulares representativas
q3_ext = atan2(d4,a3);                                % brazo extendido
q3_plg = atan2(d4,a3) - pi;                           % brazo plegado
q3_h   = 0;
q2_h   = fzero(@(x) r_fun(x,q3_h), deg2rad(45));      % r = 0 (muneca sobre eje J1)

q_extendido = [0, -pi/2, q3_ext, 0, deg2rad(45), 0];
q_plegado   = [0, -pi/2, q3_plg, 0, deg2rad(45), 0];
q_hombro    = [0, q2_h,  q3_h,   0, deg2rad(45), 0];
q_muneca    = [0, deg2rad(-70), deg2rad(40), deg2rad(30), 0, deg2rad(20)];
q_regular   = [0.3, -0.5, 0.8, 0.4, 0.7, -0.2];

fprintf('\n=== SINGULARIDADES (Jacobiano en el centro de la muneca) ===\n');
fprintf('det(J) = a2 * r(q2,q3) * (d4 cos q3 - a3 sin q3) * (-sin q5)\n\n');
fprintf('CODO   : d4 cos q3 - a3 sin q3 = 0 -> q3 = %s grados\n', mat2str(rad2deg(q3_codo),5));
fprintf('MUNECA : sin q5 = 0                 -> q5 = %s grados\n', mat2str(rad2deg(q5_mun),5));
fprintf('HOMBRO : r(q2,q3) = 0 (curva en el plano q2-q3). Ej: q3 = 0 -> q2 = %.2f grados\n', rad2deg(q2_h));
fprintf('Alcance maximo en el plano a1+a2+sqrt(a3^2+d4^2) = %.4f m (datasheet: 2.032 m)\n\n', a1+a2+hypot(a3,d4));

%% 1. VERIFICACION DE LAS FORMULAS CONTRA EL TOOLBOX
R.tool = transl(0,0,-0.075);      % punto de referencia = centro de muneca
N  = 300;
ev = zeros(N,1);  ew = zeros(N,1);
for k = 1:N
    qk = qlim(:,1)' + rand(1,6).*(qlim(:,2)-qlim(:,1))';
    J  = R.jacob0(qk);
    ev(k) = abs(det(J(1:3,1:3)) - detJv(qk(2),qk(3)));
    ew(k) = abs(det(J(4:6,4:6)) - detJw(qk(5)));
end
fprintf('Verificacion con %d configuraciones aleatorias:\n', N);
fprintf('  max |det(Jv) toolbox - formula| = %.2e\n', max(ev));
fprintf('  max |det(Jw) toolbox - formula| = %.2e\n\n', max(ew));

%% 2. FIGURA 1: MAPA DE SINGULARIDADES EN EL ESPACIO ARTICULAR (q2,q3)
q2v = linspace(qlim(2,1), qlim(2,2), 300);
q3v = linspace(qlim(3,1), qlim(3,2), 500);
[Q2,Q3] = meshgrid(q2v,q3v);
Rg = r_fun(Q2,Q3);
DV = abs(detJv(Q2,Q3));

figure('Name','Mapa de singularidades del brazo (q2,q3)','Position',[50 50 900 650]);
imagesc(rad2deg(q2v), rad2deg(q3v), DV); set(gca,'YDir','normal');
colormap(parula); cb = colorbar; ylabel(cb,'|det(Jv)|'); hold on;

contour(rad2deg(Q2), rad2deg(Q3), Rg, [0 0], 'r', 'LineWidth', 3);        % hombro
for k = 1:numel(q3_codo)                                                  % codo
    plot(rad2deg([q2v(1) q2v(end)]), rad2deg(q3_codo(k))*[1 1], 'w--', 'LineWidth', 2.5);
end
hS = plot(NaN,NaN,'r-','LineWidth',3);
hE = plot(NaN,NaN,'w--','LineWidth',2.5);

pts = [rad2deg(q_extendido(2)) rad2deg(q_extendido(3));
       rad2deg(q_plegado(2))   rad2deg(q_plegado(3));
       rad2deg(q_hombro(2))    rad2deg(q_hombro(3))];
lbl = {'A','B','C'};
for k = 1:3
    plot(pts(k,1),pts(k,2),'wo','MarkerFaceColor','k','MarkerSize',9);
    text(pts(k,1)+3, pts(k,2)+8, lbl{k}, 'Color','w','FontWeight','bold','FontSize',13);
end
xlabel('q2 [grados]'); ylabel('q3 [grados]');
title('Brillo = |det(Jv)|. Zonas oscuras = cerca de singularidad');
legend([hS hE],{'Hombro: r = 0','Codo: d4 cos q3 - a3 sin q3 = 0'},'Location','northeast');

%% 3. FIGURA 2: CONFIGURACIONES SINGULARES EN 3D
poses   = {q_extendido, q_plegado, q_hombro, q_muneca};
titulos = {sprintf('A) Codo extendido (q3 = %.1f grados)', rad2deg(q3_ext)), ...
           sprintf('B) Codo plegado (q3 = %.1f grados)',   rad2deg(q3_plg)), ...
           'C) Hombro: muneca sobre el eje de J1 (r = 0)', ...
           'D) Muneca: q5 = 0 (ejes de J4 y J6 alineados)'};

figure('Name','Configuraciones singulares','Position',[60 60 1200 850]);
for k = 1:4
    subplot(2,2,k);
    dibujar_pose(dh, off, poses{k}, pb, k);
    title(titulos{k});
end

%% 4. FIGURA 3: SINGULARIDAD DE CODO EN EL PLANO DEL BRAZO
q2p = -pi/2;                                    % brazo horizontal para dibujar
q3s = [deg2rad(40), q3_ext, q3_plg];
figure('Name','Singularidad de codo (plano del brazo)','Position',[80 80 1250 420]);
for k = 1:3
    subplot(1,3,k);
    dibujar_plano_codo(a2,a3,d4,q2p,q3s(k));
end

%% 5. FIGURA 4: SINGULARIDAD DE HOMBRO (VISTA SUPERIOR)
r_reg  = r_fun(-pi/2, 0);
r_sing = r_fun(q2_h, q3_h);
figure('Name','Singularidad de hombro (vista superior)','Position',[100 100 950 450]);
subplot(1,2,1); dibujar_vista_superior(r_reg,  d3); title(sprintf('Configuracion regular: r = %.3f m', r_reg));
subplot(1,2,2); dibujar_vista_superior(r_sing, d3); title(sprintf('Singular: r = %.3f m', r_sing));

%% 6. RANGO Y VALOR SINGULAR MINIMO EN CADA CONFIGURACION
fprintf('%-18s %12s %12s %6s\n','Configuracion','det(J)','sigma_min','rango');
etiq  = {'codo extendido','codo plegado','hombro (r=0)','muneca (q5=0)','regular'};
todas = {q_extendido, q_plegado, q_hombro, q_muneca, q_regular};
for k = 1:5
    J = R.jacob0(todas{k});
    s = svd(J);
    fprintf('%-18s %12.2e %12.2e %6d\n', etiq{k}, det(J), min(s), rank(J,1e-8));
end

%% ======================= FUNCIONES LOCALES =======================

function [P,Z] = cadena_cinematica(dh, off, q, pb)
% P(i+1,:) = origen del frame i ; Z(i+1,:) = eje z del frame i (en el mundo)
    T = eye(4); T(1:3,4) = pb(:);
    P = zeros(7,3); Z = zeros(7,3);
    P(1,:) = T(1:3,4)';  Z(1,:) = T(1:3,3)';
    for i = 1:6
        th = dh(i,1) + q(i) + off(i);
        d = dh(i,2); a = dh(i,3); al = dh(i,4);
        A = [cos(th) -sin(th)*cos(al)  sin(th)*sin(al) a*cos(th);
             sin(th)  cos(th)*cos(al) -cos(th)*sin(al) a*sin(th);
             0        sin(al)          cos(al)         d;
             0        0                0               1];
        T = T*A;
        P(i+1,:) = T(1:3,4)';
        Z(i+1,:) = T(1:3,3)';
    end
end

function dibujar_pose(dh, off, q, pb, tipo)
    [P,Z] = cadena_cinematica(dh, off, q, pb);
    hold on; grid on;
    h0 = plot3(P(:,1),P(:,2),P(:,3),'-o','Color',[0.3 0.3 0.3],'LineWidth',3, ...
               'MarkerFaceColor',[1 0.7 0],'MarkerSize',6);
    h1 = plot3(pb(1)*[1 1], pb(2)*[1 1], [0 3.2], 'k--', 'LineWidth',1.2);
    W  = P(5,:);                                   % centro de muneca (origen frame 4)
    h2 = plot3(W(1),W(2),W(3),'rp','MarkerSize',16,'MarkerFaceColor','r');
    H = [h0 h1 h2];
    N = {'cadena cinematica','eje de J1','centro de muneca'};
    if tipo <= 2
        h3 = plot3(P(2:3,1),P(2:3,2),P(2:3,3),'b-','LineWidth',6);
        h4 = plot3(P([3 5],1),P([3 5],2),P([3 5],3),'r-','LineWidth',6);
        H = [H h3 h4];  N = [N {'brazo (a2)','antebrazo efectivo'}];
    elseif tipo == 4
        h3 = quiver3(W(1),W(2),W(3),0.9*Z(4,1),0.9*Z(4,2),0.9*Z(4,3),0,'r','LineWidth',3,'MaxHeadSize',0.5);
        h4 = quiver3(W(1),W(2),W(3),0.6*Z(6,1),0.6*Z(6,2),0.6*Z(6,3),0,'b','LineWidth',3,'MaxHeadSize',0.5);
        H = [H h3 h4];  N = [N {'eje de J4 (z3)','eje de J6 (z5)'}];
    end
    legend(H, N, 'Location','northeast', 'FontSize',8);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    axis equal; xlim([-1.3 2.5]); ylim([-1.3 1.3]); zlim([0 3.3]);
    view(35,20);
end

function dibujar_plano_codo(a2, a3, d4, q2, q3)
% Plano del brazo: J2 en el origen, brazo (azul) y antebrazo efectivo (rojo)
    Rot  = @(t) [cos(t) -sin(t); sin(t) cos(t)];
    perp = @(v) [-v(2); v(1)];
    O = [0;0];
    E = Rot(q2)*[0; a2];                       % posicion de J3
    W = E + Rot(q2+q3)*[d4; a3];               % centro de muneca
    val = d4*cos(q3) - a3*sin(q3);
    hold on; grid on; axis equal;
    hb = plot([O(1) E(1)],[O(2) E(2)],'b-','LineWidth',6);
    hr = plot([E(1) W(1)],[E(2) W(2)],'r-','LineWidth',6);
    plot(O(1),O(2),'ko','MarkerFaceColor','k','MarkerSize',9);
    plot(E(1),E(2),'ko','MarkerFaceColor','y','MarkerSize',9);
    plot(W(1),W(2),'rp','MarkerFaceColor','r','MarkerSize',15);
    % direcciones de velocidad de la muneca por dq2 y dq3 (normalizadas)
    v2 = perp(W-O); v2 = 0.9*v2/norm(v2);
    v3 = perp(W-E); v3 = 0.6*v3/norm(v3);
    h2 = quiver(W(1),W(2),v2(1),v2(2),0,'Color',[0 0.6 0],'LineWidth',2.5,'MaxHeadSize',0.6);
    h3 = quiver(W(1),W(2),v3(1),v3(2),0,'Color',[0.8 0 0.8],'LineWidth',2.5,'MaxHeadSize',0.6);
    if abs(val) < 1e-9
        u = (W-O)/norm(W-O);
        plot([W(1) W(1)+0.8*u(1)],[W(2) W(2)+0.8*u(2)],'k--','LineWidth',1.5);
        text(W(1)+0.8*u(1), W(2)+0.15, 'direccion sin movilidad', ...
             'HorizontalAlignment','center','FontSize',8);
    end
    legend([hb hr h2 h3], {'brazo (a_2)','antebrazo efectivo', 'vel. por dq_2','vel. por dq_3'}, ...
           'Location','south','FontSize',7);
    title(sprintf('q_3 = %.1f^{\\circ}   d_4cos q_3 - a_3sin q_3 = %.3f', rad2deg(q3), val));
    xlabel('direccion radial [m]'); ylabel('altura [m]');
    xlim([-1.2 2.6]); ylim([-1.6 1.8]);
end

function dibujar_vista_superior(r, d3)
% Vista desde arriba: eje de J1 en el origen, X = direccion radial del plano del brazo
    hold on; grid on; axis equal;
    W  = [r; -d3];                              % muneca en el sistema girado con q1
    v1 = [-W(2); W(1)]; v1 = v1/norm(v1);       % direccion de velocidad por dq1
    plot(0,0,'ks','MarkerFaceColor','k','MarkerSize',10);
    plot([0 W(1)],[0 W(2)],'k:');
    plot(W(1),W(2),'rp','MarkerFaceColor','r','MarkerSize',15);
    h1 = quiver(W(1),W(2),0.9*v1(1),0.9*v1(2),0,'Color',[0 0.6 0],'LineWidth',2.5,'MaxHeadSize',0.6);
    h2 = quiver(W(1),W(2), 0.6,0,0,'Color',[0.8 0 0.8],'LineWidth',2.5,'MaxHeadSize',0.6);
         quiver(W(1),W(2),-0.6,0,0,'Color',[0.8 0 0.8],'LineWidth',2.5,'MaxHeadSize',0.6);
    legend([h1 h2], {'vel. por dq_1','vel. por dq_2, dq_3 (solo radial)'}, ...
           'Location','southoutside','FontSize',8);
    xlabel('direccion radial (plano del brazo) [m]');
    ylabel('direccion tangencial [m]');
    xlim([-1 2]); ylim([-1.2 1.2]);
end