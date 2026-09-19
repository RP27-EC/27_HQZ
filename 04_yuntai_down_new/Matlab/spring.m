%% 氮气弹簧前馈补偿 - 腿长与补偿力矩的三次多项式拟合
% 拟合模型: T_comp = a*L^3 + b*L^2 + c*L + d

clear; clc; close all;

%% ==================== 1. 示例数据（请替换为你的实际数据） ====================

% 示例腿长数据 (m)
L_data = [0.242
0.13
0.114
0.163
0.143
0.299
0.260 
0.310 
0.152 
0.218



];

% 示例补偿力矩数据 (Nm) - 根据实际物理特性设定
T_data = [-0.5
4.18
5.08
2.26
3.35
-0.9
-0.35
-1
3.2
0.1


];

%% ==================== 2. 三次多项式拟合 ====================

% 方法1: 使用 polyfit 进行三次拟合 (推荐)
p = polyfit(L_data, T_data, 3);  % p = [a, b, c, d]

% 提取系数
a = p(1);  % L^3 系数
b = p(2);  % L^2 系数
c = p(3);  % L^1 系数
d = p(4);  % 常数项

fprintf('========== 三次多项式拟合结果 ==========\n');
fprintf('拟合方程: T_comp = %.6f*L^3 + %.6f*L^2 + %.6f*L + %.6f\n', a, b, c, d);
fprintf('系数 a (L^3): %.6f\n', a);
fprintf('系数 b (L^2): %.6f\n', b);
fprintf('系数 c (L^1): %.6f\n', c);
fprintf('系数 d (常数): %.6f\n', d);

%% ==================== 3. 计算拟合优度 ====================

% 计算拟合值
T_fit = polyval(p, L_data);

% 计算 R^2 (决定系数)
SS_res = sum((T_data - T_fit).^2);      % 残差平方和
SS_tot = sum((T_data - mean(T_data)).^2);  % 总平方和
R_squared = 1 - SS_res/SS_tot;

% 计算 RMSE (均方根误差)
RMSE = sqrt(mean((T_data - T_fit).^2));

fprintf('\n========== 拟合质量评估 ==========\n');
fprintf('R^2 (决定系数): %.6f (越接近1越好)\n', R_squared);
fprintf('RMSE (均方根误差): %.6f Nm\n', RMSE);

%% ==================== 4. 可视化 ====================

% 生成平滑曲线数据
L_smooth = linspace(min(L_data), max(L_data), 100);
T_smooth = polyval(p, L_smooth);

figure('Name', '氮气弹簧前馈补偿拟合', 'Position', [100 100 800 500]);

% 绘制原始数据点
plot(L_data, T_data, 'ro', 'MarkerSize', 10, 'LineWidth', 2, ...
    'DisplayName', '原始数据');
hold on;

% 绘制拟合曲线
plot(L_smooth, T_smooth, 'b-', 'LineWidth', 2, 'DisplayName', '三次拟合曲线');

% 添加拟合方程文本
eq_text = sprintf('T_{comp} = %.4fL^3 + %.4fL^2 + %.4fL + %.4f\nR^2 = %.4f', ...
    a, b, c, d, R_squared);
text(0.5, 0.9*max(T_data), eq_text, 'FontSize', 11, 'BackgroundColor', 'white', ...
    'EdgeColor', 'black', 'Units', 'normalized', 'Position', [0.15, 0.75]);

xlabel('腿长 L (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('补偿力矩 T_{comp} (Nm)', 'FontSize', 12, 'FontWeight', 'bold');
title('氮气弹簧前馈补偿 - 腿长与补偿力矩关系', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
legend('Location', 'best');
set(gca, 'FontSize', 11);

%% ==================== 5. 前馈补偿计算函数 ====================

% 定义函数句柄，便于实时计算
compensateTorque = @(L) a*L.^3 + b*L.^2 + c*L + d;

% 示例：计算特定腿长下的补偿力矩
test_L = 0.5;  % 测试腿长
test_T = compensateTorque(test_L);
fprintf('\n========== 前馈补偿计算示例 ==========\n');
fprintf('当腿长 L = %.3f m 时，补偿力矩 T_comp = %.4f Nm\n', test_L, test_T);

%% ==================== 6. 保存结果 ====================

% 保存拟合系数到结构体
fit_result.coefficients = struct('a', a, 'b', b, 'c', c, 'd', d);
fit_result.R_squared = R_squared;
fit_result.RMSE = RMSE;
fit_result.equation = sprintf('%.6f*L^3 + %.6f*L^2 + %.6f*L + %.6f', a, b, c, d);

% 保存到文件
save('nitrogen_spring_feedforward_fit.mat', 'fit_result');
fprintf('\n拟合结果已保存到: nitrogen_spring_feedforward_fit.mat\n');

%% ==================== 7. 实时补偿函数（用于控制器） ====================

function T_comp = nitrogenSpringFeedforward(L, coeffs)
    %NITROGENSPRINGFEEDFORWARD 计算氮气弹簧前馈补偿力矩
    %   输入:
    %       L      - 腿长 (m)，可以是标量或向量
    %       coeffs - 拟合系数 [a, b, c, d]
    %   输出:
    %       T_comp - 补偿力矩 (Nm)
    
    a = coeffs(1); b = coeffs(2); c = coeffs(3); d = coeffs(4);
    T_comp = a*L.^3 + b*L.^2 + c*L + d;
end

% 使用示例
coeffs = [a, b, c, d];
T_realtime = nitrogenSpringFeedforward(0.55, coeffs);
fprintf('实时计算: L=0.55m 时, T_comp=%.4f Nm\n', T_realtime);