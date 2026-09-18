clear; clc; close all;

%% ==================== 输入配置区 ====================
CONFIG.RUN_MODE = 'COMPARE';      % 'BASE_ONLY' | 'MOD_ONLY' | 'COMPARE' 
CONFIG.LEG_SELECTION = 'R_Leg';   % 'R_Leg' | 'L_Leg' 

% 电机返回角度：弧度，范围 [-pi, pi]，顺时针变小
MOTOR_DATA.ch0_now = deg2rad(-170.95921);   % 已改为弧度输入
MOTOR_DATA.ch1_now = deg2rad(-7.1415443);   

% 零点校准参数（弧度）
ZERO_CAL = struct(); 
ZERO_CAL.R_F_HORIZON_ANGLE = -1.2387563 + 1.9595;   
ZERO_CAL.R_B_HORIZON_ANGLE = 0.127753735 + 0.5574;  

% 机构参数
PARAMS_BASE = struct(); 
PARAMS_BASE.l1 = 0.215;    % 前大腿
PARAMS_BASE.l2 = 0.258;    % 前小腿
PARAMS_BASE.l3 = 0.258;    % 后小腿
PARAMS_BASE.l4 = 0.215;    % 后大腿
PARAMS_BASE.l5 = 0.0;      % 髋关节间距

% 修改参数（对比用）
PARAMS_MOD = PARAMS_BASE;
PARAMS_MOD.l1 = 0.213;
PARAMS_MOD.l2 = 0.260;

%% ==================== 计算区 ====================
fprintf('========================================\n'); 
fprintf('    五连杆运动学 | 电机弧度±pi | 顺时针变小\n'); 
fprintf('========================================\n\n'); 

[zero_cal, order_corr] = get_leg_params(CONFIG.LEG_SELECTION, ZERO_CAL); 

% 电机角度直接使用弧度输入
phi1_motor = MOTOR_DATA.ch0_now;
phi4_motor = MOTOR_DATA.ch1_now;

% 方向匹配：顺时针角度变小
phi1_raw = order_corr.F_TIME * phi1_motor + order_corr.F_ORDER * zero_cal.F_HORIZON; 
phi4_raw = order_corr.B_TIME * phi4_motor + order_corr.B_ORDER * zero_cal.B_HORIZON; 

phi1 = angle_normalize(phi1_raw); 
phi4 = angle_normalize(phi4_raw); 

result_b = link_calc_vertical(phi1, phi4, PARAMS_BASE); 
result_m = link_calc_vertical(phi1, phi4, PARAMS_MOD); 

%% ==================== 绘图 ====================
if strcmp(CONFIG.RUN_MODE, 'BASE_ONLY')
    draw_leg_vertical_coord(result_b, PARAMS_BASE, '基准 | 腿竖直朝下');
    
elseif strcmp(CONFIG.RUN_MODE, 'MOD_ONLY')
    draw_leg_vertical_coord(result_m, PARAMS_MOD, '修改 | 腿竖直朝下');
    
elseif strcmp(CONFIG.RUN_MODE, 'COMPARE')
    figure(1);
    draw_leg_vertical_coord(result_b, PARAMS_BASE, '基准 | 腿竖直朝下');
    
    figure(2);
    draw_leg_vertical_coord(result_m, PARAMS_MOD, '修改 | 腿竖直朝下');
    
    figure(3);
    draw_leg_compare_vertical(result_b, result_m);
    
    print_compare(result_b, result_m);
end

fprintf('\n计算完成\n');

%% ==================== 函数 ====================
function res = link_calc_vertical(phi1, phi4, p)
    l1 = p.l1; l2 = p.l2; l4 = p.l4; l5 = p.l5;
    x0 = 0; y0 = 0;
    
    xB = l1 * cos(phi1);
    yB = l1 * sin(phi1);
    xC = xB + l2 * cos(phi1 + phi4);
    yC = yB + l2 * sin(phi1 + phi4);
    
    res.coord.xc = xC;
    res.coord.yc = yC;
    res.joint.xB = xB;
    res.joint.yB = yB;
    
    % 腿角定义：0°向右，90°向上，-90°竖直向下
    res.angle.vir_phi0 = atan2(yC, xC);
end

% 绘制腿竖直朝下 + 角度制坐标系
function draw_leg_vertical_coord(res, p, tit)
    x0 = 0; y0 = 0;
    xB = res.joint.xB; yB = res.joint.yB;
    xC = res.coord.xc; yC = res.coord.yc;
    
    clf; hold on; axis equal; grid on; box on;
    set(gca,'FontSize',10,'FontName','微软雅黑');
    
    % 机身水平
    plot([-0.3 0.3],[0 0],'k-','LineWidth',3,'DisplayName','机身水平');
    
    % 连杆
    plot([x0 xB],[y0 yB],'b-o','LineWidth',2.5,'MarkerSize',7);
    plot([xB xC],[yB yC],'g-o','LineWidth',2.5,'MarkerSize',7);
    plot(xC,yC,'ro','MarkerSize',9,'DisplayName','足端');
    
    % ========== 角度制坐标系 ==========
    quiver(0,0,0.20,0,'Color','r','LineWidth',2);
    text(0.22,0,'0°','Color','r','FontSize',12,'FontWeight','bold');
    
    quiver(0,0,0,0.20,'Color','r','LineWidth',2);
    text(0,0.22,'90°','Color','r','FontSize',12,'FontWeight','bold');
    
    quiver(0,0,0,-0.20,'Color','r','LineWidth',2);
    text(0,-0.25,'-90° 腿竖直朝下','Color','r','FontSize',12,'FontWeight','bold');
    
    % 当前腿角
    phi_deg = rad2deg(res.angle.vir_phi0);
    text(-0.28,0.20,sprintf('腿角 = %.1f°',phi_deg),...
         'FontSize',12,'Color','blue','FontWeight','bold');
    
    xlabel('X (m)'); ylabel('Y (m)');
    title(tit,'FontWeight','bold');
    xlim([-0.35 0.35]); ylim([-0.45 0.30]);
    legend('Location','southeast');
    hold off;
end

% 对比图
function draw_leg_compare_vertical(rb, rm)
    clf; hold on; axis equal; grid on; box on;
    set(gca,'FontSize',10,'FontName','微软雅黑');
    
    plot([-0.3 0.3],[0 0],'k-','LineWidth',3,'DisplayName','机身水平');
    
    % 基准
    x0=0;y0=0;
    xBb=rb.joint.xB; yBb=rb.joint.yB;
    xCb=rb.coord.xc; yCb=rb.coord.yc;
    plot([x0 xBb],[y0 yBb],'b-o','LineWidth',2);
    plot([xBb xCb],[yBb yCb],'b-','LineWidth',2);
    plot(xCb,yCb,'bo','MarkerSize',8,'DisplayName','基准足端');
    
    % 修改
    xBm=rm.joint.xB; yBm=rm.joint.yB;
    xCm=rm.coord.xc; yCm=rm.coord.yc;
    plot([x0 xBm],[y0 yBm],'r--o','LineWidth',2);
    plot([xBm xCm],[yBm yCm],'r--','LineWidth',2);
    plot(xCm,yCm,'ro','MarkerSize',8,'DisplayName','修改足端');
    
    % 偏差线
    plot([xCb xCm],[yCb yCm],'m-','LineWidth',2,'DisplayName','偏差向量');
    
    % 坐标系
    quiver(0,0,0.20,0,'k','LineWidth',1.5); text(0.22,0,'0°','FontSize',11);
    quiver(0,0,0,0.20,'k','LineWidth',1.5); text(0,0.22,'90°','FontSize',11);
    quiver(0,0,0,-0.20,'k','LineWidth',1.5); text(0,-0.25,'-90°','FontSize',11);
    
    % 偏差
    dx = (xCm-xCb)*1000;
    dy = (yCm-yCb)*1000;
    dphi = rad2deg(rm.angle.vir_phi0 - rb.angle.vir_phi0);
    dist = sqrt(dx^2+dy^2);
    
    text(-0.28,0.20,sprintf('\\DeltaX = %.1f mm',dx),'FontSize',11);
    text(-0.28,0.15,sprintf('\\DeltaY = %.1f mm',dy),'FontSize',11);
    text(-0.28,0.10,sprintf('\\Delta腿角 = %.1f°',dphi),'FontSize',11,'Color','red');
    text(-0.28,0.05,sprintf('偏差距离 = %.1f mm',dist),'FontSize',11,'Color','red');
    
    xlabel('X (m)'); ylabel('Y (m)');
    title('腿竖直朝下 | 基准 vs 修改 对比','FontWeight','bold');
    xlim([-0.35 0.35]); ylim([-0.45 0.30]);
    legend('Location','southeast');
    hold off;
end

function [zero_cal, order_corr] = get_leg_params(leg, zc)
    if strcmp(leg, 'R_Leg')
        zero_cal.F_HORIZON = zc.R_F_HORIZON_ANGLE; 
        zero_cal.B_HORIZON = zc.R_B_HORIZON_ANGLE; 
        order_corr.F_TIME = -1; 
        order_corr.B_TIME = 1; 
        order_corr.F_ORDER = 1; 
        order_corr.B_ORDER = -1; 
    else
        zero_cal.F_HORIZON = zc.L_F_HORIZON_ANGLE; 
        zero_cal.B_HORIZON = zc.L_B_HORIZON_ANGLE; 
        order_corr.F_TIME = 1; 
        order_corr.B_TIME = -1; 
        order_corr.F_ORDER = -1; 
        order_corr.B_ORDER = 1; 
    end
end

function phi = angle_normalize(p)
    phi = mod(p + pi, 2*pi) - pi; 
end

function print_compare(rb, rm)
    fprintf('\n========== 对比结果 ==========\n');
    fprintf('基准腿角：%.2f°\n', rad2deg(rb.angle.vir_phi0));
    fprintf('修改腿角：%.2f°\n', rad2deg(rm.angle.vir_phi0));
    fprintf('角度偏差：%.2f°\n', rad2deg(rm.angle.vir_phi0 - rb.angle.vir_phi0));
    dx = (rm.coord.xc - rb.coord.xc)*1000;
    dy = (rm.coord.yc - rb.coord.yc)*1000;
    fprintf('位置偏差：ΔX=%.1fmm  ΔY=%.1fmm\n', dx, dy);
end