clc
close all

% CINEMÁTICA DIRECTA SCARA BOLDO

a1 = 3;
a2 = 2;
d1 = 4;
d3 = 1;
d4 = 2;
% MEDIDAS HIPOTÉTICAS

DH = [0, d1, a1, 0,  0, 0;
      0, 0,  a2, pi, 0, 0;
      0, 0,  0,  0,  1, d3;
      0, d4, 0,  0,  0, 0];

R = SerialLink(DH)
R.name = 'SCARA';
R.plot([0, 0, 0, 0], 'workspace', [-8 8 -8 8 -8 8])
R.qlim = [-pi pi; -pi*2/3 pi*2/3; 0 2; -pi pi];
R.teach

q0 = [0; 0; 0; 0];
 
Tr1 = CinemDirecta(DH, q0)
Tr2 = R.fkine(q0)
%R.links(4).A(q0(4))
function [T] = CinemDirecta (DH, qi)
    T = eye(4);
    for i = 1:size(DH, 1)
        if DH(i,5) == 0
            T = T * transf(qi(i), DH(i, 2), DH(i, 3), DH(i, 4), DH(i, 5), DH(i, 6));
        else 
            T = T * transf(DH(i, 1), qi(i), DH(i, 3), DH(i, 4), DH(i, 5), DH(i, 6));
        end
    end
end
function [Tr] = transf(theta, d, a, alfa, gamma, offset)

    if gamma == 0
        Tr = trotz(theta + offset) * transl(0, 0, d) * transl(a, 0, 0) * trotx(alfa);
        %disp(Tr)
    else 
        Tr = trotz(theta) * transl(0, 0, d + offset) * transl(a, 0, 0) * trotx(alfa);
        %disp(Tr)
    end
end

