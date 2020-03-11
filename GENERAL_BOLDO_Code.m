clc
close all

% MEDIDAS 
a1 = 0.400;
a2 = 0.200;
d1 = 0.13637;
d3 = 0.02737;
d4 = 0.059;
DH = [0, d1, a1, 0,  0, 0;
      0, 0,  a2, pi, 0, 0;
      0, 0,  0,  0,  1, d3;
      0, d4, 0,  0,  0, 0];
  
R = SerialLink(DH)
R.name = 'BOLDO';
R.plot([0, pi*2/3, 0, 0], 'workspace', [-0.8 0.8 -0.8 0.8 -0.08 0.14])
% R.plot3d([0, pi*2/3, 0, 0], 'path', pwd, 'workspace', [-0.1 0.2 -0.655 0.2 -0.1 0.2])
lim = [-3*pi/2, 3*pi/2; -pi*2/3, pi*2/3; 0, 0.050; -pi, pi];
R.qlim = lim;
R.teach;
%PROBAMOS QUÉ SUCEDE AL INGRESAR UN VECTOR CON COORDENADAS ARTICULARES
%QUE EXCEDEN LOS LÍMITES ESTABLECIDOS
Q_prueba_CD = [10, -20, 30, -18];
T_0_4 = Cinem_Directa(Q_prueba_CD(1, :), a1, a2, d1, d3, d4, lim);
%SE ESTABLECEN PARÁMETROS PARA COMENZAR
Q_h = [0, pi*2/3, 0, 0]; %POSICIÓN DE HOMING
%COMPROBAMOS EL DETERMINANTE, RANGO E INDEPENDENCIA DE COORDENADAS EN EL 
%JACOBIANO DE NUESTRO VECTOR Q_0 QUE ES PARTICULARMENTE UNA SINGULARIDAD
Q_0 = [0, 0, 0, 0];
V_cart = [0.1, 0.1, 0.1, 0.1]; %VELOCIDADES CARTESIANAS QUE SE UTILIZAN PARA PROBAR LA 
J = Jacobiano(Q_0, a1, a2);    %FUNCIÓN Veloc_art CON EL JACOBIANO
V_art = Veloc_art(Q_0, a1, a2, V_cart);
%DECLARAMOS ALGUNAS VARIABLES PARA EL CÁLCULO DE TRAJECTORIAS
T_cont = 0:0.05:4;
T_vect = [1, 1, 1, 1];
paso = 0.05;
%SIMULAMOS UN CAMINO DE PICK AND PLACE
Q_ant = Q_h;
pos_h = [0.3, 0.1732051, 0.05, 2.0944]; %HOMING
pos = [-0.4, -0.2, 0, 1.5708;         %PICK DE LA PLATAFORMA
       0.5732051, -0.1, 0, 2.0944;     %PLACE EN LA BANDEJA
       pos_h];                        %HOMING, IMAGINANDO QUE TERMINÓ TODA SU TAREA
puntos = length(pos(:, 1));

for i = 1 : puntos
%     Q = generador_Q(lim); %TAMBIÉN PODEMOS GENERAR DATOS DE ENTRADA ALEATORIA QUE CUMPLAN CON LOS LIMITES ART
    %PASAMOS A COORD CARTESIANAS QUE SERÁN NUESTROS VERDADEROS DATOS DE ENTRADA X, Y, Z, PHI
%      Q = Q_traj(i+1, :)
%      T_0_4 = Cinem_Directa(Q(1, :), a1, a2, d1, d3, d4, lim); 
%      phi = atan2(T_0_4(2, 1), T_0_4(1, 1))
    pos_destino = pos(i, :)
    x = pos(i, 1);
    y = pos(i, 2);
    z = pos(i, 3);
    phi = pos(i, 4);
    Q_ant

    %CALCULAMOS CINEMÁTICA INVERSA
    Q = Cinem_Inversa(a1, a2, d1, d3, d4, T_0_4, x, y, z, phi, lim)
    %OBTENEMOS DOS SOLUCIONES, ENTONCES CALCULAMOS CUÁL NOS COMBIENE,
    %SEGÚN LA DISTANCIA ENTRE ESTE VECTOR ARTICULAR A TOMAR Y EL ANTERIOR
    Q_final = Best_Q(Q, Q_ant, a1, a2)
    %CALCULAMOS LA TRAYECTORIA DESEADA
    T_final = Cinem_Directa(Q_final(1, :), a1, a2, d1, d3, d4, lim); 
    [q, qd, qdd, T] = Trayectoria(Q_ant, Q_final, T_cont, T_vect, a1, a2, d1, d3, d4, lim, paso);
    s_T = size(T);
    %REALIZAMOS LA ANIMACIÓN
    R.animate(q)
    %CALCULAMOS EL JACOBIANO
    J = Jacobiano(Q_final, a1, a2);
    plot_ellipse(J(1:2, :)*J(1:2, :)', [x, y, z])

    %CALCULAMOS VELOCIDADES ARTICULARES CON EL JACOBIANO, A PARTIR DE
    %VELOCIDADES CARTESIANAS CTES DEFINIDAS AL PRINCIPIO
    V_art = Veloc_art(Q_final, a1, a2, V_cart);
    %TAMBIÉN SE PUEDEN CALCULAR TODAS LAS VELOCIDADES CARTESIANAS DE TODA
    %LA TRAYECTORIA REALIZADA
    %ACTUALIZAMOS VARIABLES
    Q_ant = Q_final;
    fprintf('\n');
end

%CINEMÁTICA INVERSA
function [Q] = Cinem_Inversa(a1, a2, d1, d3, d4, T_0_4, x, y, z, phi, lim)

    if nargin == 7
        x = T_0_4(1, 4);
        y = T_0_4(2, 4);
        z = T_0_4(3, 4);
    end 
    p_0_2 = [x; y; d1; 1];  %VECTOR 0P2
    Q_1 = zeros(2, 4);
    Q_1(1, 1) = atan2(p_0_2(2), p_0_2(1)) + acos(((p_0_2(1))^2 + (p_0_2(2))^2 + a1^2 - a2^2) / (2*a1*sqrt((p_0_2(1))^2 + (p_0_2(2))^2)));
    Q_1(2, 1) = atan2(p_0_2(2), p_0_2(1)) - acos(((p_0_2(1))^2 + (p_0_2(2))^2 + a1^2 - a2^2) / (2*a1*sqrt((p_0_2(1))^2 + (p_0_2(2))^2)));
    %A CONTINUACIÓN SE DETERMINA SI TENDREMOS 2, 3 O 4 SOLUCIONES
    flag = 0;
    flag1 = 0;
    for i = 1 : 2
        if (Q_1(i, 1) > 2*pi - lim(1, 2))
            flag = flag + 1; flag1 = 1;
            Q_1(i + 2, 1) = Q_1(i, 1) - 2*pi;
        elseif (Q_1(i, 1) < -2*pi - lim(1, 1))
            flag = flag + 1; flag1 = -1;
            Q_1(i + 2, 1) = Q_1(i, 1) + 2*pi;
        end
    end
    if (flag == 0)
        Q = zeros(2, 4);
        for k = 1 : length(Q(:, 1))
            Q(k, 1) = Q_1(k, 1);
        end
    elseif (flag == 1)&&(flag1 == 1)
        Q = zeros(3, 4);
        for k = 1 : length(Q(:, 1))
            Q(k, 1) = Q_1(k, 1);
        end
    elseif (flag == 1)&&(flag1 == -1)
         Q = zeros(3, 4);
         for k = 1 : length(Q(:, 1))
            Q(k, 1) = Q_1(k, 1);
         end
         Q(3, 1) = Q_1(4, 1);
    elseif (flag == 2)
        Q = zeros(4, 4);
        for k = 1 : length(Q(:, 1))
            Q(k, 1) = Q_1(k, 1);
        end
    end
    %CALCULAMOS EL RESTO DE LAS COORDENADAS ARTICULARES
    for i = 1 : length(Q(:, 1))
        Q(i, 2) = atan2((-p_0_2(1)*sin(Q(i, 1)) + p_0_2(2)*cos(Q(i, 1))), (p_0_2(1)*cos(Q(i, 1)) + p_0_2(2)*sin(Q(i, 1)) - a1));
        Q(i, 3) = d1 - d3 - d4 - z;
        %SI LOS PARÁMETROS RECIBIDOS SON 6, ES DECIR QUE EL DATO DE ENTRADA ES
        %LA MATRIZ DE TRANSFORMACIÓN T_0_4, SINO, LAS ENTRADAS SON LAS 
        %COORDENADAS CARTESIANAS X, Y, Z, PHI
        if nargin == 7
            Q(i, 4) = atan2((T_0_4(1, 1)*sin(Q(i, 1) + Q(i, 2)) - T_0_4(1, 2)*cos(Q(i, 1) + Q(i, 2))), (T_0_4(1, 1)*cos(Q(i, 1) + Q(i, 2)) + T_0_4(1, 2)*sin(Q(i, 1) + Q(i, 2))));
        else
            Q(i, 4) = -(phi - Q(i, 1) - Q(i, 2));
        end  
    end
    
end

%CINEMÁTICA DIRECTA
function [T, flag] = Cinem_Directa(Q, a1, a2, d1, d3, d4, lim)
    flag = 1;
    %VERIFICAMOS SI NUESTRO VECTOR ARTICULAR DE ENTRADA ESTÁ DENTRO DE LOS
    %LIMITES DE TRABAJO DE NUESTRO ROBOT
    for i = 1 : 4
        if (Q(1, i) < lim(i, 1))
            flag = 0; Q(1, i) = lim(i, 1);
        elseif (Q(1, i) > lim(i, 2))
            flag = 0; Q(1, i) = lim(i, 2);
        end
    end
    %SI NO ESTÁ DENTRO DE LOS LÍMITES, SE TOMA LA POSICIÓN ART MÁXIMA PARA
    %ESA ARTICULACIÓN
    if flag == 0
        %fprintf('La posición deseada sobrepasa los límites de trabajo\n');
        %fprintf('Q_posible = [ '); fprintf('%f ', Q); fprintf(']\n');
    end
    T = [cos(Q(1, 1) + Q(1, 2) - Q(1, 4)),  sin(Q(1, 1) + Q(1, 2) - Q(1, 4)),  0, a2*cos(Q(1, 1) + Q(1, 2)) + a1*cos(Q(1, 1));
         sin(Q(1, 1) + Q(1, 2) - Q(1, 4)), -cos(Q(1, 1) + Q(1, 2) - Q(1, 4)),  0, a2*sin(Q(1, 1) + Q(1, 2)) + a1*sin(Q(1, 1));
                                        0,                                 0, -1,                      d1 - Q(1, 3) - d3 - d4;
                                        0,                                 0,  0,                                           1];
                                    
  
end

%JACOBIANO
function [J, DJ] = Jacobiano(Q_final, a1, a2)


    J = [-a1*sin(Q_final(1, 1)) - a2*sin(Q_final(1, 1) + Q_final(1, 2)), -a2*sin(Q_final(1, 1) + Q_final(1, 2)),  0,  0;
          a1*cos(Q_final(1, 1)) + a2*cos(Q_final(1, 1) + Q_final(1, 2)),  a2*cos(Q_final(1, 1) + Q_final(1, 2)),  0,  0;
                                                                      0,                                      0, -1,  0;
                                                                      1,                                      1,  0, -1];
    %MOSTRAMOS POR PANTALLA ALGUNOS DATOS IMPORTANTES
    rango = rank(J);
    %fprintf('Rango del Jacobiano = %d\n', rango);
    DJ = det(J);    
    %fprintf('Determinante del Jacobiano = %f\n', DJ);
    %jsingu(J)
end

%VELOCIDAD ARTICULAR
function [V_art] = Veloc_art(Q, a1, a2, V_cart)
    %PARA CALCULAR VELOCIDADES ARTICULARES EN PUNTOS SINGULARES USAMOS LA
    %PSEUDOINVERSA DE LA MATRIZ JACOBIANA
    [J, DJ] = Jacobiano(Q, a1, a2);
    if DJ ~= 0
        J_inv = [cos(Q(1, 1) + Q(1, 2))/(a1*sin(Q(1, 2))),                             sin(Q(1, 1) + Q(1, 2))/(a1*sin(Q(1, 2))),                            0,  0
                 -(a2*cos(Q(1, 1) + Q(1, 2)) + a1*cos(Q(1, 1)))/(a1*a2*sin(Q(1, 2))),  -(a2*sin(Q(1, 1) + Q(1, 2)) + a1*sin(Q(1, 1)))/(a1*a2*sin(Q(1, 2))),  0,  0
                 0,                                                                    0,                                                                  -1,  0
                 -cos(Q(1, 1))/(a2*sin(Q(1, 2))),                                     -sin(Q(1, 1))/(a2*sin(Q(1, 2))),                                      0, -1];
    else
%       q1 = Q(1, 1); 
%       q2 = Q(1, 2);
        J_inv = pinv(J);
%       J_inv = [((a2*cos(q1 + q2) + a1*cos(q1))*(2*a2^2*cos(q1 + q2)*sin(q1 + q2) + 2*a1^2*cos(q1)*sin(q1) + a1*a2*cos(q1 + q2)*sin(q1) + a1*a2*sin(q1 + q2)*cos(q1)))/(a1^2*a2^2*cos(q1 + q2)^2*sin(q1)^2 + a1^2*a2^2*sin(q1 + q2)^2*cos(q1)^2 - 2*a1^2*a2^2*cos(q1 + q2)*sin(q1 + q2)*cos(q1)*sin(q1)) - (2*(a2*sin(q1 + q2) + a1*sin(q1))*(a1^2*cos(q1)^2 + a1*a2*cos(q1 + q2)*cos(q1) + a2^2*cos(q1 + q2)^2))/(a1^2*a2^2*cos(q1 + q2)^2*sin(q1)^2 + a1^2*a2^2*sin(q1 + q2)^2*cos(q1)^2 - 2*a1^2*a2^2*cos(q1 + q2)*sin(q1 + q2)*cos(q1)*sin(q1)) - cos(q1)/(a2*cos(q1 + q2)*sin(q1) - a2*sin(q1 + q2)*cos(q1)), (2*(a2*cos(q1 + q2) + a1*cos(q1))*(a1^2*sin(q1)^2 + a1*a2*sin(q1 + q2)*sin(q1) + a2^2*sin(q1 + q2)^2))/(a1^2*a2^2*cos(q1 + q2)^2*sin(q1)^2 + a1^2*a2^2*sin(q1 + q2)^2*cos(q1)^2 - 2*a1^2*a2^2*cos(q1 + q2)*sin(q1 + q2)*cos(q1)*sin(q1)) - sin(q1)/(a2*cos(q1 + q2)*sin(q1) - a2*sin(q1 + q2)*cos(q1)) - ((a2*sin(q1 + q2) + a1*sin(q1))*(2*a2^2*cos(q1 + q2)*sin(q1 + q2) + 2*a1^2*cos(q1)*sin(q1) + a1*a2*cos(q1 + q2)*sin(q1) + a1*a2*sin(q1 + q2)*cos(q1)))/(a1^2*a2^2*cos(q1 + q2)^2*sin(q1)^2 + a1^2*a2^2*sin(q1 + q2)^2*cos(q1)^2 - 2*a1^2*a2^2*cos(q1 + q2)*sin(q1 + q2)*cos(q1)*sin(q1)),  0, (cos(q1)*(a2*sin(q1 + q2) + a1*sin(q1)))/(a2*cos(q1 + q2)*sin(q1) - a2*sin(q1 + q2)*cos(q1)) - (sin(q1)*(a2*cos(q1 + q2) + a1*cos(q1)))/(a2*cos(q1 + q2)*sin(q1) - a2*sin(q1 + q2)*cos(q1)) + 1;
%                (a2*cos(q1 + q2)*(2*a2^2*cos(q1 + q2)*sin(q1 + q2) + 2*a1^2*cos(q1)*sin(q1) + a1*a2*cos(q1 + q2)*sin(q1) + a1*a2*sin(q1 + q2)*cos(q1)))/(a1^2*a2^2*cos(q1 + q2)^2*sin(q1)^2 + a1^2*a2^2*sin(q1 + q2)^2*cos(q1)^2 - 2*a1^2*a2^2*cos(q1 + q2)*sin(q1 + q2)*cos(q1)*sin(q1)) - cos(q1)/(a2*cos(q1 + q2)*sin(q1) - a2*sin(q1 + q2)*cos(q1)) - (2*a2*sin(q1 + q2)*(a1^2*cos(q1)^2 + a1*a2*cos(q1 + q2)*cos(q1) + a2^2*cos(q1 + q2)^2))/(a1^2*a2^2*cos(q1 + q2)^2*sin(q1)^2 + a1^2*a2^2*sin(q1 + q2)^2*cos(q1)^2 - 2*a1^2*a2^2*cos(q1 + q2)*sin(q1 + q2)*cos(q1)*sin(q1)),                               (2*a2*cos(q1 + q2)*(a1^2*sin(q1)^2 + a1*a2*sin(q1 + q2)*sin(q1) + a2^2*sin(q1 + q2)^2))/(a1^2*a2^2*cos(q1 + q2)^2*sin(q1)^2 + a1^2*a2^2*sin(q1 + q2)^2*cos(q1)^2 - 2*a1^2*a2^2*cos(q1 + q2)*sin(q1 + q2)*cos(q1)*sin(q1)) - sin(q1)/(a2*cos(q1 + q2)*sin(q1) - a2*sin(q1 + q2)*cos(q1)) - (a2*sin(q1 + q2)*(2*a2^2*cos(q1 + q2)*sin(q1 + q2) + 2*a1^2*cos(q1)*sin(q1) + a1*a2*cos(q1 + q2)*sin(q1) + a1*a2*sin(q1 + q2)*cos(q1)))/(a1^2*a2^2*cos(q1 + q2)^2*sin(q1)^2 + a1^2*a2^2*sin(q1 + q2)^2*cos(q1)^2 - 2*a1^2*a2^2*cos(q1 + q2)*sin(q1 + q2)*cos(q1)*sin(q1)),                                0, (a2*sin(q1 + q2)*cos(q1))/(a2*cos(q1 + q2)*sin(q1) - a2*sin(q1 + q2)*cos(q1)) - (a2*cos(q1 + q2)*sin(q1))/(a2*cos(q1 + q2)*sin(q1) - a2*sin(q1 + q2)*cos(q1)) + 1;
%                 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                -1,  0;
%                 cos(q1)/(a2*cos(q1 + q2)*sin(q1) - a2*sin(q1 + q2)*cos(q1)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       sin(q1)/(a2*cos(q1 + q2)*sin(q1) - a2*sin(q1 + q2)*cos(q1)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       0, -1];
    end
    
    V_art = J_inv * V_cart';
    %SE CALCULA CON VELOCIDAD CARTESIANA CTE DEL VECTOR V_cart
    fprintf('Velocidad articular = [ '); 
    fprintf('%f ', V_art); fprintf(']\n');

end

%DISTANCIA ENTRE POS ART ACTUAL Y POS ART ANTERIOR
function [dist1, dist2] = distancia_q(Q, Q_ant, a1, a2)
    dist1 = abs(a1*(Q(1, 1) - Q_ant(1, 1)));
    dist2 = abs(a2*(Q(1, 2) - Q_ant(1, 2)));
end

%CALCULAMOS LA POSICIÓN FINAL MÁS CERCANA A LA ANTERIOR
function [Q_final] = Best_Q(Q, Q_ant, a1, a2)
    dist_q1 = zeros(2, 1);
    dist_q2 = zeros(2, 1);
    for j = 1 : length(Q(:, 1))
        [dist_q1(j, 1), dist_q2(j, 1)] = distancia_q (Q(j, :), Q_ant(1, :), a1, a2);
    end
    [ord_dq1, ind_dg1] = sort(dist_q1);
    for i = ind_dg1
        if ord_dq1(i, 1) > dist_q2(ind_dg1(i), 1)
            A(i, 1) = ord_dq1(i, 1);
        else
            A(i, 1) = dist_q2(ind_dg1(i), 1);
        end
    end
    [B, I] = min(A);
    Q_final = Q(ind_dg1(I), :);
end

%CÁLCULO DE TRAYECTORIA
function [q1, dq1, ddq1, T] = Trayectoria(Q_ant, Q_final, T_cont, T_vect, a1, a2, d1, d3, d4, lim, paso)
    v_max = 1;
    q = jtraj(Q_ant, Q_final, T_cont);
    s_T = size(T_cont);
    t_1 = round(s_T(1, 2)/4);
    t_2 = t_1*2;
    t_3 = t_1*3;

    if (Q_ant(1, 3) < 0.0001)&&(Q_ant(1, 3) > -0.0001)
         %DE HOMING A OTRA POSICIÓN
        Q_i = [Q_ant(1, 1), Q_ant(1, 2), 0.01, Q_ant(1, 4)];
        Q_med1 = [q(t_1, 1), q(t_1, 2), lim(3, 2)/4, q(t_1, 4)];
        Q_med2 = [q(t_2, 1), q(t_2, 2), lim(3, 2)/2, q(t_2, 4)];
        Q_med3 = [q(t_3, 1), q(t_3, 2), lim(3, 2)*3/4, q(t_3, 4)];
        Q_f = [Q_final(1, 1), Q_final(1, 2), 0.04, Q_final(1, 4)];
    elseif (Q_final(1, 3) < 0.0001)&&(Q_final(1, 3) > -0.0001)
        %DE CUALQUIER POSICIÓN AL HOMING
        Q_i = [Q_ant(1, 1), Q_ant(1, 2), 0.04, Q_ant(1, 4)];
        Q_med1 = [q(t_1, 1), q(t_1, 2), lim(3, 2)*3/4, q(t_1, 4)];
        Q_med2 = [q(t_2, 1), q(t_2, 2), lim(3, 2)/2, q(t_2, 4)];
        Q_med3 = [q(t_3, 1), q(t_3, 2), lim(3, 2)/4, q(t_3, 4)];
        Q_f = [Q_final(1, 1), Q_final(1, 2), 0.01, Q_final(1, 4)];
    else
        %ENTRE POSICIONES
        Q_i = [Q_ant(1, 1), Q_ant(1, 2), 0.04, Q_ant(1, 4)];
        Q_med1 = [q(t_1, 1), q(t_1, 2), lim(3, 2)/4, q(t_1, 4)];
        Q_med2 = [q(t_2, 1), q(t_2, 2), lim(3, 1), q(t_2, 4)];
        Q_med3 = [q(t_3, 1), q(t_3, 2), lim(3, 2)/4, q(t_3, 4)];
        Q_f = [Q_final(1, 1), Q_final(1, 2), 0.04, Q_final(1, 4)];
    end
    %TRAYECTORIA
    T_ant = Cinem_Directa(Q_ant, a1, a2, d1, d3, d4, lim);
    T_i = Cinem_Directa(Q_i, a1, a2, d1, d3, d4, lim);
    T_f = Cinem_Directa(Q_f, a1, a2, d1, d3, d4, lim);
    T_final = Cinem_Directa(Q_final, a1, a2, d1, d3, d4, lim);
    T_ci = ctraj(T_ant, T_i, 10); 
    T_cf = ctraj(T_f, T_final, 10);
    s_Tif = size(T_ci);
    phi_ci = atan2(T_ci(2, 1), T_ci(1, 1));
    phi_cf = atan2(T_cf(2, 1), T_cf(1, 1));
    q1_cci = zeros(s_Tif(1, 3), 4);
    q1_ccf = zeros(s_Tif(1, 3), 4);
    Q_antci = Q_ant;
    Q_antcf = Q_f;
    for h = 1:s_Tif(1, 3)
        q1_i = Cinem_Inversa(a1, a2, d1, d3, d4, T_ci(:, :, h), T_ci(1, 4, h), T_ci(2, 4, h), T_ci(3, 4, h), phi_ci, lim);
        q1_f = Cinem_Inversa(a1, a2, d1, d3, d4, T_cf(:, :, h), T_cf(1, 4, h), T_cf(2, 4, h), T_cf(3, 4, h), phi_cf, lim);
        q1_cci(h, :) = Best_Q(q1_i, Q_antci, a1, a2);
        q1_ccf(h, :) = Best_Q(q1_f, Q_antcf, a1, a2);
        Q_antci = q1_cci(h, :);
        Q_antcf = q1_ccf(h, :);
    end
    P1 = [Q_med1; Q_med2; Q_med3; Q_f];
    q11 = mstraj(P1, [], T_vect, Q_i, paso, 0.9);
    q1 = [q1_cci ; q11; q1_ccf];
    q12 = jtraj(Q_i, Q_f, length(q1(:, 1))-20);
    for h = 1:10
        q124_ci(h, :) = Q_i;
        q124_cf(h, :) = Q_f;
    end
    q124 = [q124_ci; q12; q124_cf];
    q1(:, 1) = q124(:, 1);
    q1(:, 2) = q124(:, 2);
    q1(:, 4) = q124(:, 4);
    %VELOCIDAD
    dq1 = diff(q1)/paso;
    %ACELERACIÓN
    ddq1 = diff(q1, 2)/((paso)^2);
    %VERIFICAMOS QUE NO SE EXCEDA LA VELOCIDAD LÍMITE, SI LO HACE
    %AUMENTAMOS EL TIEMPO
    v_max_ob = max(abs(dq1));
    if v_max_ob > v_max
        T_vect = T_vect*1.5;
        [q1, dq1, ddq1] = Trayectoria(Q_ant, Q_final, T_cont, T_vect, a1, a2, d1, d3, d4, lim, paso);
    end
    %PARA GRAFICAR
    final = length(q1)*paso-paso;
    T = 0 : paso : final;
    s_T = size(T);
    q1_cc = zeros(s_T(1, 2), 3);
    for h = 1:s_T(1, 2)
        T_coord_c = Cinem_Directa(q1(h, :), a1, a2, d1, d3, d4, lim);
        q1_cc(h, 1) = T_coord_c(1, 4);
        q1_cc(h, 2) = T_coord_c(2, 4);
        q1_cc(h, 3) = T_coord_c(3, 4);
    end
    
%     figure(2)
%     grid on
%     qplot(T, q1);
%     hold on
%     title('Trayectorias')
%     plot(0, Q_ant(1, 3), '*r')
%     plot(0.5, Q_i(1, 3), '*r')
%     plot(1.5, Q_med1(1, 3), '*r')
%     plot(2.5, Q_med2(1, 3), '*r')
%     plot(3.5, Q_med3(1, 3), '*r')
%     plot(4.5, Q_f(1, 3), '*r')
%     plot(5, Q_final(1, 3), '*r')
%     hold off
%     figure(3)
%     qplot(T(1, 1:s_T(1, 2)-1), dq1);
%     title('Velocidades')
%     figure(4)
%     qplot(T(1, 1:s_T(1, 2)-2), ddq1);
%     title('Aceleraciones')
% 
%     figure(5)
%     grid on
%     plot(T, q1(:, 2));
%     title('Trayectoria q2')
%     hold on
%     plot(0, Q_ant(1, 2), '*r')
%     plot(0.5, Q_i(1, 2), '*r')
%     plot(1.5, Q_med1(1, 2), '*r')
%     plot(2.5, Q_med2(1, 2), '*r')
%     plot(3.5, Q_med3(1, 2), '*r')
%     plot(4.5, Q_f(1, 2), '*r')
%     plot(5, Q_final(1, 2), '*r')
%     hold off
%     plot(T(1, 1:s_T(1, 2)-1), dq1(:, 2), 'm');
%     title('Velocidad q2')
%     figure(7)
%     plot(T(1, 1:s_T(1, 2)-2), ddq1(:, 2), 'g');
%     title('Aceleración q2')
%     
%     figure(8)
%     grid on
%     plot(T, q1(:, 3));
%     title('Trayectoria q3')
%     hold on
%     plot(0, Q_ant(1, 3), '*r')
%     plot(0.5, Q_i(1, 3), '*r')
%     plot(1.5, Q_med1(1, 3), '*r')
%     plot(2.5, Q_med2(1, 3), '*r')
%     plot(3.5, Q_med3(1, 3), '*r')
%     plot(4.5, Q_f(1, 3), '*r')
%     plot(5, Q_final(1, 3), '*r')
%     hold off
%     figure(9)
%     plot(T(1, 1:s_T(1, 2)-1), dq1(:, 3), 'm');
%     title('Velocidad q3')
%     figure(10)
%     plot(T(1, 1:s_T(1, 2)-2), ddq1(:, 3), 'g');
%     title('Aceleración q3')
%      
%     figure(11)
%     grid on
%     plot(T, q1(:, 4));
%     title('Trayectoria q4')
%     hold on
%     plot(0, Q_ant(1, 4), '*r')
%     plot(0.5, Q_i(1, 4), '*r')
%     plot(1.5, Q_med1(1, 4), '*r')
%     plot(2.5, Q_med2(1, 4), '*r')
%     plot(3.5, Q_med3(1, 4), '*r')
%     plot(4.5, Q_f(1, 4), '*r')
%     plot(5, Q_final(1, 4), '*r')
%     hold off
%     figure(12)
%     plot(T(1, 1:s_T(1, 2)-1), dq1(:, 4), 'm');
%     title('Velocidad q4')
%     figure(13)
%     plot(T(1, 1:s_T(1, 2)-2), ddq1(:, 4), 'g');
%     title('Aceleración q4')
%     
%     figure(14)
%     grid on
%     plot(T, q1(:, 1));
%     title('Trayectoria q1')
%     hold on
%     plot(0, Q_ant(1, 1), '*r')
%     plot(0.5, Q_i(1, 1), '*r')
%     plot(1.5, Q_med1(1, 1), '*r')
%     plot(2.5, Q_med2(1, 1), '*r')
%     plot(3.5, Q_med3(1, 1), '*r')
%     plot(4.5, Q_f(1, 1), '*r')
%     plot(5, Q_final(1, 1), '*r')
%     hold off
%     figure(15)
%     plot(T(1, 1:s_T(1, 2)-1), dq1(:, 1), 'm');
%     title('Velocidad q1')
%     figure(16)
%     plot(T(1, 1:s_T(1, 2)-2), ddq1(:, 1), 'g');
%     title('Aceleración q1')
    
%     figure(17)
%     
%     plot3(q1_cc(:, 1), q1_cc(:, 2), q1_cc(:, 3), '.r');
%     title('Trayectoria del EFECTOR FINAL (Coord. Cartesianas)')
%     grid on
%     axis([-0.65 0.65 -0.65 0.65 0 0.05])
    
end

%GENERA POSICIONES DE PICK AND PLACE ALEATORIAS
function [Q] = generador_Q(lim)
    r = zeros(1, 4);
    for i = 1 : 4
        r(1, i) = lim(i, 1) + rand(1, 1)*(lim(i, 2) - lim(i, 1));
    end
    Q = [r(1, 1), r(1, 2), lim(3, 2), r(1, 4)];
end