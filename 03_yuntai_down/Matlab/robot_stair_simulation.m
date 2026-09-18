function robot_stair_descent_only()
    %% ========== 初始化参数 ==========
    clear; close all; clc;
    
    % 物理参数（来自car_info.h）
    g = 9.81;
    R_wheel = 0.058;        % 轮半径 5.8cm
    M_body = 19.8578;       % 机体质量
    M_leg = 1.3066;         % 摆杆质量
    M_wheel = 0.5895;       % 轮子质量
    l_cg = 0.02925;         % 机体质心偏移
    
    % 腿长范围
    L_min = 0.125;          % 最短腿长 12.5cm
    L_max = 0.325;          % 最长腿长 32.5cm
    L_nominal = 0.225;      %  nominal腿长 22.5cm
    
    % 地形参数（倒L型障碍）
    curb_height = 0.02;     % 坎高 2cm
    step_height = 0.20;     % 台阶高 20cm
    step_depth = 0.20;      % 台阶深 20cm
    
    % 仿真参数
    dt = 0.001;             % 仿真步长1ms
    t_total = 4;            % 总仿真时间4秒（足够下台阶）
    t = 0:dt:t_total;
    N = length(t);
    
    % 初始状态 [theta; theta_dot; x; x_dot; phi; phi_dot]
    X = zeros(6, N);
    X(:,1) = [0.05; 0; 0.1; 0.8; 0; 0];  % 初始在坎前，有前进速度
    
    % 腿长轨迹
    L_t = L_nominal * ones(1, N);
    
    %% ========== LQR控制器设计 ==========
    Q = diag([300, 1, 20, 5, 2500, 5]);
    R = diag([3, 0.25]);
    
    % 预计算不同腿长的K矩阵
    leg_lengths = L_min:0.005:L_max;
    K_gain = zeros(2, 6, length(leg_lengths));
    
    for i = 1:length(leg_lengths)
        L = leg_lengths(i);
        [A, B] = get_linearized_model(L, M_body, M_leg, M_wheel, R_wheel, l_cg, g);
        try
            K_gain(:,:,i) = lqr(A, B, Q, R);
        catch
            K_gain(:,:,i) = zeros(2,6);
        end
    end
    
    K_leg_lengths = leg_lengths;
    K_data = K_gain;
    
    %% ========== 地形定义 ==========
    terrain = @(x) get_terrain_height(x, curb_height, step_height, step_depth);
    terrain_slope = @(x) get_terrain_slope(x, curb_height, step_height, step_depth);
    
    %% ========== 预规划腿长轨迹（下台阶专用） ==========
    for k = 1:N
        x_pos = X(3,1) + 0.8 * t(k);  % 预估位置
        
        if x_pos < 0 - 0.05
            % 坎前：正常腿长
            L_t(k) = L_nominal;
        elseif x_pos >= 0 - 0.05 && x_pos < 0.15
            % 上坎过程：稍微缩腿
            L_t(k) = L_nominal - 0.02;
        elseif x_pos >= 0.15 && x_pos < 0.20
            % 坎边缘：准备下台阶，缩到最短
            L_t(k) = L_min + 0.01;
        elseif x_pos >= 0.20 && x_pos < 0.35
            % 下台阶过程：快速伸腿缓冲
            progress = (x_pos - 0.20) / 0.15;
            L_t(k) = (L_min + 0.01) + progress * (L_nominal - L_min - 0.01);
        else
            % 已经下完台阶，保持正常
            L_t(k) = L_nominal;
        end
    end
    
    L_t = smoothdata(L_t, 'gaussian', 50);
    
    %% ========== 主仿真循环 ==========
    U = zeros(2, N);
    wheel_pos = zeros(2, N);
    wheel_pos(:,1) = [X(3,1); terrain(X(3,1))];
    contact_force = zeros(1, N);
    
    % 记录仿真实际结束点
    actual_end_k = N;
    descent_complete = false;
    
    figure('Name', 'Robot Descending L-Shape Obstacle', 'Position', [50 50 1400 900]);
    
    for k = 1:N-1
        current_x = X(3,k);
        current_L = L_t(k);
        
        % 检查是否已经完成下台阶（轮子到达底部平台）
        if current_x > 0.45 && ~descent_complete
            % 已经下完台阶，记录结束点并退出
            actual_end_k = k;
            descent_complete = true;
            fprintf('下台阶完成！位置: %.3f m, 时间: %.3f s\n', current_x, t(k));
            break;  % 结束仿真循环
        end
        
        z_ground = terrain(current_x);
        slope = terrain_slope(current_x);
        
        wheel_pos(1,k) = current_x;
        wheel_pos(2,k) = z_ground + R_wheel;
        
        K = get_K_interpolated(current_L, K_leg_lengths, K_data);
        
        % 参考状态：保持直立，匀速前进
        X_ref = [0; 0; current_x + 0.15; 0.4; 0; 0];
        
        U_fb = -K * (X(:,k) - X_ref);
        U_ff = [M_body * g * slope * R_wheel; 0];
        U(:,k) = U_fb + U_ff;
        
        U(1,k) = max(min(U(1,k), 15), -15);
        U(2,k) = max(min(U(2,k), 8), -8);
        
        [X(:,k+1), contact_force(k)] = robot_dynamics_constrained(...
            X(:,k), U(:,k), current_L, dt, ...
            M_body, M_leg, M_wheel, R_wheel, l_cg, g, ...
            z_ground, slope);
        
        % 3D可视化
        if mod(k, 10) == 0 || k == 1
            clf;
            
            % 子图1：3D动画
            subplot(2, 3, [1 2 4 5]);
            draw_robot_3d_descent(X(:,k), wheel_pos(:,k), current_L, ...
                                 R_wheel, terrain, curb_height, step_height, step_depth);
            
            title(sprintf('Time: %.2f s | Leg: %.1f cm | Pos: %.2f m | 下台阶中...', ...
                  t(k), current_L*100, current_x), 'FontSize', 12, 'FontWeight', 'bold');
            
            xlim([-0.2 0.6]); 
            ylim([-0.25 0.25]); 
            zlim([-0.25 0.1]);
            view(40, 25);
            grid on; axis equal;
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            
            % 添加高度标注
            hold on;
            plot3([0.2 0.2], [0.12 0.12], [0 0.02], 'm-', 'LineWidth', 2);
            text(0.21, 0.12, 0.01, '2cm坎', 'Color', 'm', 'FontSize', 9);
            plot3([0.2 0.2], [0.15 0.15], [0.02 -0.18], 'r-', 'LineWidth', 2);
            text(0.21, 0.15, -0.08, '20cm台阶', 'Color', 'r', 'FontSize', 9);
            
            % 子图2：状态变量
            subplot(2, 3, 3);
            plot(t(1:k), X(1,1:k)*180/pi, 'r-', 'LineWidth', 1.5); hold on;
            plot(t(1:k), X(5,1:k)*180/pi, 'b-', 'LineWidth', 1.5);
            line([t(1) t(end)], [10 10], 'Color', [1 0.7 0.7], 'LineStyle', '--');
            line([t(1) t(end)], [-10 -10], 'Color', [1 0.7 0.7], 'LineStyle', '--');
            legend('\theta (摆角)', '\phi (机体角)', 'Location', 'best');
            xlabel('Time (s)'); ylabel('Angle (deg)');
            title('机体姿态'); grid on; ylim([-15 15]);
            
            % 子图3：控制输入
            subplot(2, 3, 6);
            plot(t(1:k), U(1,1:k), 'g-', 'LineWidth', 1.5); hold on;
            plot(t(1:k), U(2,1:k), 'm-', 'LineWidth', 1.5);
            legend('轮力矩', '髋关节力矩', 'Location', 'best');
            xlabel('Time (s)'); ylabel('力矩 (N·m)');
            title('控制输入'); grid on;
            
            drawnow;
        end
    end
    
    %% ========== 最终结果显示 ==========
    % 截取实际仿真数据
    t_actual = t(1:actual_end_k);
    X_actual = X(:,1:actual_end_k);
    U_actual = U(:,1:actual_end_k);
    wheel_pos_actual = wheel_pos(:,1:actual_end_k);
    L_t_actual = L_t(1:actual_end_k);
    
    % 最终分析图
    figure('Name', '下台阶完成 - 结果分析', 'Position', [100 100 1400 800]);
    
    % 地形剖面+轨迹
    subplot(2,3,1);
    x_fine = linspace(-0.2, 0.6, 500);
    z_fine = arrayfun(terrain, x_fine);
    area(x_fine, z_fine*100, 'FaceColor', [0.8 0.8 0.6], 'EdgeColor', 'k', 'LineWidth', 2);
    hold on;
    plot(wheel_pos_actual(1,:), (wheel_pos_actual(2,:)-R_wheel)*100, 'r-', 'LineWidth', 2.5);
    plot(wheel_pos_actual(1,end), (wheel_pos_actual(2,end)-R_wheel)*100, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlabel('X (m)'); ylabel('高度 (cm)');
    title('地形剖面与轮心轨迹');
    grid on;
    text(0.1, 1, '2cm坎', 'HorizontalAlignment', 'center', 'BackgroundColor', 'w');
    text(0.3, -10, '20cm台阶', 'HorizontalAlignment', 'center', 'Color', 'r', 'BackgroundColor', 'w');
    text(wheel_pos_actual(1,end), (wheel_pos_actual(2,end)-R_wheel)*100-3, '终点', 'Color', 'r', 'HorizontalAlignment', 'center');
    
    % 水平位置
    subplot(2,3,2);
    plot(t_actual, X_actual(3,:), 'b-', 'LineWidth', 1.5);
    hold on;
    line([t_actual(1) t_actual(end)], [0.45 0.45], 'Color', [0.5 0.8 0.5], 'LineStyle', '--');
    text(t_actual(end)-0.3, 0.47, '下台阶完成线', 'Color', [0 0.5 0]);
    xlabel('时间 (s)'); ylabel('位置 (m)');
    title('水平位移'); grid on;
    
    % 水平速度
    subplot(2,3,3);
    plot(t_actual, X_actual(4,:), 'r-', 'LineWidth', 1.5);
    xlabel('时间 (s)'); ylabel('速度 (m/s)');
    title('前进速度'); grid on;
    
    % 角度
    subplot(2,3,4);
    plot(t_actual, X_actual(1,:)*180/pi, 'r-', 'LineWidth', 1.5); hold on;
    plot(t_actual, X_actual(5,:)*180/pi, 'b-', 'LineWidth', 1.5);
    legend('摆杆角', '机体角');
    xlabel('时间 (s)'); ylabel('角度 (deg)');
    title('姿态角度'); grid on;
    
    % 角速度
    subplot(2,3,5);
    plot(t_actual, X_actual(2,:)*180/pi, 'r-', 'LineWidth', 1.5); hold on;
    plot(t_actual, X_actual(6,:)*180/pi, 'b-', 'LineWidth', 1.5);
    legend('摆杆角速度', '机体角速度');
    xlabel('时间 (s)'); ylabel('角速度 (deg/s)');
    title('角速度'); grid on;
    
    % 腿长变化
    subplot(2,3,6);
    plot(t_actual, L_t_actual*100, 'k-', 'LineWidth', 2);
    xlabel('时间 (s)'); ylabel('腿长 (cm)');
    title('腿长变化策略'); grid on;
    ylim([10 30]);
    
    % 添加阶段标注
    hold on;
    yl = ylim;
    line([0 0], yl, 'Color', [0.5 0.5 1], 'LineStyle', '--');
    text(0.02, yl(2)-1, '上坎', 'Color', 'b', 'FontSize', 9);
    line([0.2 0.2], yl, 'Color', [1 0.5 0.5], 'LineStyle', '--');
    text(0.22, yl(2)-1, '下台阶', 'Color', 'r', 'FontSize', 9);
    
    % 显示完成信息
    fprintf('\n========== 下台阶仿真完成 ==========\n');
    fprintf('总时间: %.3f s\n', t_actual(end));
    fprintf('最终水平位置: %.3f m\n', X_actual(3,end));
    fprintf('最终高度: %.3f m (相对于地面)\n', terrain(X_actual(3,end)));
    fprintf('最大摆角: %.2f deg\n', max(abs(X_actual(1,:)))*180/pi);
    fprintf('最大机体角: %.2f deg\n', max(abs(X_actual(5,:)))*180/pi);
    fprintf('====================================\n');
    
    % 最终3D状态图
    figure('Name', '下台阶完成状态', 'Position', [200 200 800 600]);
    draw_robot_3d_descent(X_actual(:,end), wheel_pos_actual(:,end), L_t_actual(end), ...
                         R_wheel, terrain, curb_height, step_height, step_depth);
    view(45, 30);
    title(sprintf('下台阶完成 - 最终位置: %.3f m', X_actual(3,end)), 'FontSize', 14);
    xlim([-0.2 0.6]); ylim([-0.25 0.25]); zlim([-0.25 0.1]);
    grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
end

%% ========== 地形函数 ==========
function z = get_terrain_height(x, curb_h, step_h, step_d)
    if x < 0
        z = 0;
    elseif x < step_d
        z = curb_h;
    else
        z = curb_h - step_h;
    end
end

function slope = get_terrain_slope(x, curb_h, step_h, step_d)
    if x < 0 || (x > 0 && x < step_d)
        slope = 0;
    elseif abs(x - step_d) < 0.001
        slope = inf;
    else
        slope = 0;
    end
end

%% ========== 线性化模型 ==========
function [A, B] = get_linearized_model(L, M, Mp, Mw, R, l, g)
    Iw = 0.5 * Mw * R^2;
    Ip = (1/12) * Mp * (2*L)^2;
    Im = (1/12) * M * (0.161^2 + 0.5359^2);
    
    A = zeros(6,6);
    B = zeros(6,2);
    
    A(1,2) = 1;
    A(3,4) = 1;
    A(5,6) = 1;
    
    M_total = M + Mp + Mw;
    denom = Ip * Im - (Mp*L*l)^2;
    
    if abs(denom) < 1e-6
        denom = 1e-6;
    end
    
    A(2,1) = (Mp*g*L*Im - Mp*L*l*M*g*l) / denom;
    A(2,5) = (-Mp*L*l*M*g*l + M*g*l*Ip) / denom;
    
    A(6,1) = (-Im*Mp*g*L + Mp*L*l*Mp*g*L) / denom;
    A(6,5) = (Mp*L*l*M*g*l - Ip*M*g*l) / denom;
    
    B(2,1) = -Im / denom;
    B(2,2) = Ip / denom;
    B(4,1) = 1 / (Iw/R + Mw*R);
    B(6,2) = 1 / Im;
end

function K = get_K_interpolated(L, leg_lengths, K_data)
    idx = find(leg_lengths >= L, 1, 'first');
    if isempty(idx)
        K = K_data(:,:,end);
    elseif idx == 1
        K = K_data(:,:,1);
    else
        alpha = (L - leg_lengths(idx-1)) / (leg_lengths(idx) - leg_lengths(idx-1));
        K = (1-alpha) * K_data(:,:,idx-1) + alpha * K_data(:,:,idx);
    end
end

%% ========== 约束动力学 ==========
function [X_next, F_contact] = robot_dynamics_constrained(X, U, L, dt, ...
    M, Mp, Mw, R, l_cg, g, z_ground, slope)
    
    theta = X(1); theta_dot = X(2);
    x = X(3); x_dot = X(4);
    phi = X(5); phi_dot = X(6);
    
    T_wheel = U(1);
    T_hip = U(2);
    
    wheel_z = z_ground + R;
    hip_x = x + L * sin(theta);
    hip_z = wheel_z + L * cos(theta);
    
    I_leg = (1/12) * Mp * (2*L)^2;
    M_total = M + Mp + Mw;
    
    tau_gravity = Mp * g * L * sin(theta);
    tau_coupling = -M * g * l_cg * sin(phi) * cos(theta);
    
    theta_ddot = (tau_gravity + tau_coupling + T_hip) / I_leg - 0.5*theta_dot;
    
    I_wheel = 0.5 * Mw * R^2;
    x_ddot = (T_wheel/R - Mp*L*theta_ddot*cos(theta) + Mp*L*theta_dot^2*sin(theta)) ...
             / (M_total + I_wheel/R^2);
    
    I_body = (1/12) * M * (0.161^2 + 0.5359^2);
    phi_ddot = (T_hip + M*g*l_cg*sin(phi)) / I_body - 0.3*phi_dot;
    
    X_next = X + dt * [theta_dot; theta_ddot; x_dot; x_ddot; phi_dot; phi_ddot];
    
    F_contact = M_total * g * cos(atan(slope)) + M_total * x_ddot * sin(atan(slope));
end

%% ========== 3D绘制 ==========
function draw_robot_3d_descent(X, wheel_pos, L, R_wheel, terrain, curb_h, step_h, step_d)
    theta = X(1);
    phi = X(5);
    x_w = wheel_pos(1);
    z_w = wheel_pos(2);
    
    hip_x = x_w + L * sin(theta);
    hip_z = z_w + L * cos(theta);
    
    body_L = 0.35;
    body_W = 0.20;
    body_H = 0.10;
    
    hold on;
    
    % 地面基础
    fill3([-0.3 0 0 -0.3], [-0.2 -0.2 0.2 0.2], [0 0 0 0], [0.7 0.7 0.7], 'EdgeColor', 'none');
    
    % 坎（2cm高）
    fill3([0 step_d step_d 0], [-0.2 -0.2 0.2 0.2], [0 0 curb_h curb_h], [0.6 0.5 0.4], 'EdgeColor', 'k', 'LineWidth', 1);
    fill3([0 step_d step_d 0], [-0.2 -0.2 -0.2 -0.2], [0 curb_h curb_h 0], [0.6 0.5 0.4], 'EdgeColor', 'k');
    fill3([0 step_d step_d 0], [0.2 0.2 0.2 0.2], [0 curb_h curb_h 0], [0.6 0.5 0.4], 'EdgeColor', 'k');
    
    % 台阶垂直面（红色警示）
    fill3([step_d step_d step_d step_d], [-0.2 -0.2 0.2 0.2], ...
          [curb_h curb_h-step_h curb_h-step_h curb_h], [0.9 0.3 0.3], 'EdgeColor', 'r', 'LineWidth', 2);
    
    % 台阶底部平台（只画一点，表示结束点）
    fill3([step_d 0.5 0.5 step_d], [-0.2 -0.2 0.2 0.2], ...
          [curb_h-step_h curb_h-step_h curb_h-step_h curb_h-step_h], [0.6 0.6 0.6], 'EdgeColor', 'k');
    
    % 轮子
    [Xc, Yc, Zc] = cylinder(R_wheel, 20);
    Xc = Xc + x_w;
    Yc = Yc * 0.05 - 0.025;
    Zc = Zc + z_w - R_wheel;
    surf(Xc, Yc, Zc, 'FaceColor', [0.2 0.4 0.8], 'EdgeColor', 'none');
    plot3([x_w x_w], [-0.03 0.03], [z_w z_w], 'k-', 'LineWidth', 3);
    
    % 腿（两段式串腿）
    mid_x = x_w + 0.5*L*sin(theta);
    mid_z = z_w + 0.5*L*cos(theta);
    plot3([x_w mid_x], [0 0], [z_w mid_z], 'g-', 'LineWidth', 6);
    plot3([mid_x hip_x], [0 0], [mid_z hip_z], 'g-', 'LineWidth', 5);
    plot3(mid_x, 0, mid_z, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'y');
    plot3(hip_x, 0, hip_z, 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
    
    % 机体
    corners_x = [-body_L/2, body_L/2, body_L/2, -body_L/2, -body_L/2];
    corners_z = [-body_H/2, -body_H/2, body_H/2, body_H/2, -body_H/2];
    
    R_body = [cos(phi) -sin(phi); sin(phi) cos(phi)];
    body_pts = R_body * [corners_x; corners_z];
    body_x_pts = body_pts(1,:) + hip_x;
    body_z_pts = body_pts(2,:) + hip_z;
    
    for y = [-body_W/2, body_W/2]
        plot3(body_x_pts, y*ones(size(body_x_pts)), body_z_pts, 'r-', 'LineWidth', 2);
        fill3(body_x_pts(1:4), y*ones(1,4), body_z_pts(1:4), 'r', 'FaceAlpha', 0.7, 'EdgeColor', 'k');
    end
    
    for i = 1:4
        plot3([body_x_pts(i) body_x_pts(i)], [-body_W/2 body_W/2], ...
              [body_z_pts(i) body_z_pts(i)], 'r-', 'LineWidth', 1);
    end
    
    % 质心
    cg_x = hip_x + 0.05*sin(phi);
    cg_z = hip_z + 0.05*cos(phi);
    plot3(cg_x, 0, cg_z, 'b*', 'MarkerSize', 15, 'LineWidth', 2);
    
    % 重力方向
    quiver3(cg_x, 0, cg_z, 0, 0, -0.1, 'k--', 'LineWidth', 1, 'MaxHeadSize', 0.5);
    
    % 坐标轴
    quiver3(x_w-0.15, -0.2, -0.15, 0.1, 0, 0, 'r', 'LineWidth', 2);
    text(x_w-0.05, -0.2, -0.15, 'X', 'Color', 'r');
    quiver3(x_w-0.15, -0.2, -0.15, 0, 0.1, 0, 'g', 'LineWidth', 2);
    text(x_w-0.15, -0.1, -0.15, 'Y', 'Color', 'g');
    quiver3(x_w-0.15, -0.2, -0.15, 0, 0, 0.1, 'b', 'LineWidth', 2);
    text(x_w-0.15, -0.2, -0.05, 'Z', 'Color', 'b');
end