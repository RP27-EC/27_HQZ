clc;clear;

global K;
%定义符号变量，不进行数值计算
syms theta thetad1 thetad2;
syms x xdot1 xdot2;
syms phi phidot1 phidot2;
syms T Tp N P Nm Pm Nf t;

syms R L Lm l Mw Mp M Iw Ip Im g 

% 中间计算部分保持不变
Nm = M*( xdot2 + (L+Lm)*thetad2*cos(theta) - (L+Lm)*(thetad1)^2*sin(theta)  - l*phidot2*cos(phi) + l*(phidot1)^2*sin(phi) );
N  = Mp*(xdot2 + L*thetad2*cos(theta) - L*(thetad1)^2*sin(theta)) + Nm;
Pm  = M*( -(L+Lm)*thetad2*sin(theta) - (L+Lm)*(thetad1)^2*cos(theta) - l*phidot2*sin(phi) - l*(phidot1)^2*cos(phi) ) + M*g;
P  = Mp*(-L*thetad2*sin(theta) - L*(thetad1)^2*cos(theta)) + Pm + Mp*g;

eq1 = xdot2 - (T - N*R)/(Iw/R +Mw*R);
eq2 = (P*L + Pm*Lm)*sin(theta) - (N*L + Nm*Lm)*cos(theta) - T + Tp -Ip*thetad2;
eq3 = Tp + Nm*l*cos(phi) + Pm*l*sin(phi) -Im*phidot2;

[xdot2_c, thetad2_c, phidot2_c] = solve([eq1,eq2,eq3], [xdot2, thetad2, phidot2]);

% 状态变量和输入变量
X = [theta; thetad1; x; xdot1; phi; phidot1];
U = [T; Tp];
Xdot = [thetad1; thetad2_c; xdot1; xdot2_c; phidot1; phidot2_c];

%通过jacobian函数得到A，B矩阵
A = jacobian(Xdot, X);
B = jacobian(Xdot, U);

%代入系统平衡点
A = subs( A,[theta; thetad1; x ;xdot1; phi; phidot1; T; Tp],[0;0;0;0;0;0;0;0]); 
B = subs( B,[theta; thetad1; x ;xdot1; phi; phidot1; T; Tp],[0;0;0;0;0;0;0;0]); 

%输出矩阵为单位阵，输出状态变量x本身
C_ballance = eye(6);
%直接传递矩阵置零，输入不直接影响输出
D_ballance = zeros(6, 2);

% 为输入矩阵和系统矩阵带入真实模型数值

%假设L=Lm
   % 固定参数
    Leg_length=0.17;
    g_val =9.81 ;   % 重力加速度，单位：m/s²
    R_val = 0.06;    % 驱动轮半径，单位：m
    L_val=Leg_length/2;
    Lm_val=Leg_length/2;
    l_val = 0.03;   % 机体重心到其转轴距离，单位：m
    Mw_val =0.5895 ;   % 驱动轮转子质量，单位：kg
    Mp_val =1.0066 ;     % 摆杆质量，单位：kg
    M_val = 20.0078;     % 机体质量，单位：kg
O1 = 0.035;
O2 = -((L_val+Lm_val)/2 - 0.02);
O3 = -((L_val+Lm_val)-0.02);
O = (M_val*O1 + Mp_val*O2 + Mw_val*O3)/(M_val+Mw_val+Mp_val);

Iw_val = (1/2)*Mw_val*R_val^2;
Ip_val = (1/12)*Mp_val*(L_val+Lm_val)^2 ;
Im_val = (1/12)*M_val*(0.161^2+0.5359^2) ;

   % Iw_val = (1/2)*Mw_val*R_val^2;     % 驱动轮转子转动惯量，单位：kg·m²
   % Im_val = (1/12)*M_val*(0.4824^2+0.165^2);     % 机体绕质心转动惯量，单位：kg·m²
   % Ip_val = (1/12)*Mp_val*(L_val+Lm_val)^2;       % 摆杆绕质心转动惯量，单位：kg·m² 

A_ballance = subs(A,[   g,    R,     L,      Lm,      l,       Mw,       Mp,     M,       Iw,        Ip,        Im], ...
                    [g_val,  R_val,  L_val,  Lm_val,  l_val,   Mw_val,   Mp_val, M_val,   Iw_val,   Ip_val,    Im_val]);
B_ballance = subs(B,[   g,    R,     L,    Lm,     l,    Mw,    Mp,    M,       Iw,        Ip,        Im], ...
                   [g_val,  R_val,  L_val,  Lm_val,  l_val,   Mw_val,   Mp_val, M_val,   Iw_val,   Ip_val,    Im_val]);
%将符号结果转化为数值结果
A_ballance = double(vpa(A_ballance));
B_ballance = double(vpa(B_ballance));

%状态误差代价
%Q = double(diag([500, 30, 20, 1, 2300, 1]));    
%输入代价
% R = double(diag([1.8, 0.23]));
 Q = double(diag([500, 30, 20, 1, 2300, 1]));   
 R = double(diag([3, 0.25]));
sys = ss(A_ballance, B_ballance, C_ballance, D_ballance);
K = lqr(sys, Q, R);


fid = fopen('output.txt', 'w');
if fid == -1
    error('无法创建或打开output.txt文件');
end


fprintf(fid, '{\\\n');
for row = 1:size(K, 1)
    fprintf(fid, '        { ');
    for col = 1:size(K, 2)
        fprintf(fid, '%.6ff', K(row, col));
        if col < size(K, 2)
            fprintf(fid, ', ');
        end
    end
    fprintf(fid, ' }');
    
    if row < size(K, 1)
        fprintf(fid, ', \\\n');
    else
        fprintf(fid, ' \\\n');
    end
end
fprintf(fid, '    }');

fclose(fid);


winopen('output.txt');
disp(K);