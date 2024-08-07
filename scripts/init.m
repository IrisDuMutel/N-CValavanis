m = 1.5;
grav = 9.81;
Jx = 0.033113;
Jz = 0.050445;
Jy = 0.033889;
Jxy = 0;
Jxz = 0;
Jyz = 0;
Ir = 4e-3; % kg m² gyr eff
I = [Jx Jxy Jxz;
    0 Jy 0;
    0 0 Jz]; % Inertias
i_1 = (I(2,2)-I(3,3))/I(1,1);
i_2 = 1/I(1,1);
i_3 = (I(3,3)-I(1,1))/I(2,2);
i_4 = 1/I(2,2);
i_5 = (I(1,1)-I(2,2))/I(3,3);
i_6 = 1/I(3,3);

% Aerodynamic drag coefs
Ax = 0.25;
Ay = 0.25;
Az = 0.25;
Aw = [Ax 0 0;
      0 Ay 0;
      0 0 Az];

%%% PID parameters
% Are written directly inside model_PID.slx

% PD params for model_SMC.slx
Kp = 0.5;
Kd = 2;
Ki = 0;

%%% State-space representation

A = [0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 0 -grav 0 0 0 0 0 0 0;
     0 0 0 grav 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0];
 
 B = [0 0 0 0;
      0 0 0 0;
      0 0 0 0;
      0 0 0 0;
      0 0 0 0;
      0 0 0 0;
      -1/m 0 0 0;
      -1/m 0 0 0;
      -1/m 0 0 0;
      0 i_2 0 0;
      0 0 i_4 0;
      0 0 0 i_6];
  
C = eye(12);
D = zeros(12,4);
     
%%%% Controllability and observability


CO = ctrb(A,B);
OB = obsv(A,C);

if rank(CO)==12
    fprintf('Controlable!\n')
end

if rank(OB)==12
    fprintf('Observable!\n');
end

% Rot_mat = [cos(th)*cos(psi) sin(phi)*sin(th)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(th)*cos(psi)+sin(phi)*sin(psi);
%     cos(th)*sin(psi) sin(phi)*sin(th)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(th)*sin(psi)-sin(phi)*cos(psi);
%     -sin(th) sin(phi)*cos(th) cos(phi)*cos(th)]; % body to NED


Q = 1*eye(12);
% 
% Q = [1/(x_max)^2 0 0 0 0 0 0 0 0 0 0 0;
%     0 1/(y_max)^2 0 0 0 0 0 0 0 0 0 0;
%     0 0 1/(h_max)^2 0 0 0 0 0 0 0 0 0;
%     0 0 0 1/(phi_max)^2 0 0 0 0 0 0 0 0;
%     0 0 0 0 1/(theta_max)^2 0 0 0 0 0 0 0;
%     0 0 0 0 0 1/(psi_max)^2 0 0 0 0 0 0;
%     0 0 0 0 0 0 1/(u_max)^2 0 0 0 0 0;
%     0 0 0 0 0 0 0 1/(v_max)^2 0 0 0 0;
%     0 0 0 0 0 0 0 0 1/(w_max)^2 0 0 0;
%     0 0 0 0 0 0 0 0 0 1 0 0;
%     0 0 0 0 0 0 0 0 0 0 1 0;
%     0 0 0 0 0 0 0 0 0 0 0 1];

R =1*eye(4,4);

 %Optimal K
[K_lqr, S, e] = lqr(A, B, Q, R);
sys = ss(A,B,C,D);
AQ = A-B*K_lqr;
eig(AQ)
