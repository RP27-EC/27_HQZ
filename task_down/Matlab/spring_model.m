%% 氮气弹簧数据 - 多种曲线拟合对比分析
clear; clc; close all;

% 实验数据
L = [0.114 0.125 0.127 0.131 0.137 0.142 0.154 0.16 0.166 0.190  0.204 0.233 0.251 0.27 0.289 0.311];
T = [5.5 4.5 4.5 4.2 4 3.5 3 2.6 1.8 1.3 0.47 0 -0.3 -0.7 -1 -1.2];

%% ========== 多种模型拟合 ==========

% 1. 线性拟合
p1 = polyfit(L, T, 1);
T1_fit = polyval(p1, L);
R2_1 = 1 - sum((T - T1_fit).^2) / sum((T - mean(T)).^2);

% 2. 二次拟合
p2 = polyfit(L, T, 2);
T2_fit = polyval(p2, L);
R2_2 = 1 - sum((T - T2_fit).^2) / sum((T - mean(T)).^2);

% 3. 三次拟合
p3 = polyfit(L, T, 3);
T3_fit = polyval(p3, L);
R2_3 = 1 - sum((T - T3_fit).^2) / sum((T - mean(T)).^2);

% 4. 指数拟合: T = a*exp(b*L) + c
% 先尝试简单指数
exp_model = fit(L', T', 'exp1');
T_exp = exp_model.a * exp(exp_model.b * L);
R2_exp = 1 - sum((T - T_exp).^2) / sum((T - mean(T)).^2);

% 5. 幂函数拟合: T = a*L^b + c
power_model = fit(L', T', 'power1');
T_power = power_model.a * L.^power_model.b;
R2_power = 1 - sum((T - T_power).^2) / sum((T - mean(T)).^2);

% 6. 对数拟合: T = a*log(L) + b
log_model = fit(L', T', 'log1');
T_log = log_model.a * log(L) + log_model.b;
R2_log = 1 - sum((T - T_log).^2) / sum((T - mean(T)).^2);

% 7. 有理分式拟合 (1阶)
rat_model = fit(L', T', 'rat11');
T_rat = rat_model.p1 ./ (L + rat_model.q1) + rat_model.p2;
R2_rat = 1 - sum((T - T_rat).^2) / sum((T - mean(T)).^2);

%% ========== 结果显示 ==========
fprintf('========================================\n');
fprintf('        不同拟合模型对比\n');
fprintf('========================================\n\n');

models = {'线性', '二次', '三次', '指数', '幂函数', '对数', '有理分式'};
R2_values = [R2_1, R2_2, R2_3, R2_exp, R2_power, R2_log, R2_rat];

fprintf('模型\t\t\tR²\t\t拟合优度评价\n');
fprintf('----------------------------------------\n');
for i = 1:length(models)
    if R2_values(i) > 0.99
        eval = '★★★ 优秀';
    elseif R2_values(i) > 0.95
        eval = '★★☆ 良好';
    elseif R2_values(i) > 0.90
        eval = '★☆☆ 一般';
    else
        eval = '☆☆☆ 较差';
    end
    fprintf('%-8s\t%.6f\t%s\n', models{i}, R2_values(i), eval);
end

%% ========== 找出最佳拟合 ==========
[R2_best, idx] = max(R2_values);
fprintf('\n【最佳拟合模型】: %s (R² = %.6f)\n', models{idx}, R2_best);

%% ========== 详细输出最佳模型 ==========
fprintf('\n========================================\n');
fprintf('        最佳模型详细参数\n');
fprintf('========================================\n');

switch idx
    case 1  % 线性
        fprintf('线性方程: T = %.6f·L + %.6f\n', p1(1), p1(2));
    case 2  % 二次
        fprintf('二次方程: T = %.6f·L² + %.6f·L + %.6f\n', p2(1), p2(2), p2(3));
    case 3  % 三次
        fprintf('三次方程: T = %.6f·L³ + %.6f·L² + %.6f·L + %.6f\n', ...
            p3(1), p3(2), p3(3), p3(4));
    case 4  % 指数
        fprintf('指数方程: T = %.6f·exp(%.6f·L)\n', exp_model.a, exp_model.b);
    case 5  % 幂函数
        fprintf('幂函数方程: T = %.6f·L^(%.6f)\n', power_model.a, power_model.b);
    case 6  % 对数
        fprintf('对数方程: T = %.6f·ln(L) + %.6f\n', log_model.a, log_model.b);
    case 7  % 有理分式
        fprintf('有理分式: T = (%.6f·L + %.6f) / (L + %.6f)\n', ...
            rat_model.p1, rat_model.p2, rat_model.q1);
end

%% ========== 可视化对比 ==========
figure('Position', [100 100 1200 500], 'Color', 'white');

% 左图: 所有拟合对比
subplot(1, 2, 1);
L_smooth = linspace(min(L)*0.9, max(L)*1.1, 200);

plot(L, T, 'ko', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', '实验数据');
hold on;

colors = {[0.8 0.2 0.2], [0.2 0.6 0.2], [0.2 0.2 0.8], [0.8 0.4 0], [0.6 0.2 0.6], [0.4 0.4 0.4], [0 0.6 0.6]};
line_styles = {'--', '-.', ':', '-', '--', '-.', ':'};

for i = 1:length(models)
    switch i
        case 1, T_plot = polyval(p1, L_smooth);
        case 2, T_plot = polyval(p2, L_smooth);
        case 3, T_plot = polyval(p3, L_smooth);
        case 4, T_plot = exp_model.a * exp(exp_model.b * L_smooth);
        case 5, T_plot = power_model.a * L_smooth.^power_model.b;
        case 6, T_plot = log_model.a * log(L_smooth) + log_model.b;
        case 7, T_plot = (rat_model.p1 * L_smooth + rat_model.p2) ./ (L_smooth + rat_model.q1);
    end
    plot(L_smooth, T_plot, 'Color', colors{i}, 'LineStyle', line_styles{i}, ...
        'LineWidth', 1.5, 'DisplayName', sprintf('%s (R²=%.3f)', models{i}, R2_values(i)));
end

xlabel('腿长 L (m)', 'FontSize', 12);
ylabel('补偿力矩 T (Nm)', 'FontSize', 12);
title('多种拟合模型对比', 'FontSize', 13);
legend('Location', 'best', 'FontSize', 9);
grid on;

% 右图: 最佳拟合细节
subplot(1, 2, 2);
plot(L, T, 'ro', 'MarkerSize', 14, 'LineWidth', 2.5, 'DisplayName', '实验数据');
hold on;

% 绘制最佳拟合曲线
switch idx
    case 1, T_best = polyval(p1, L_smooth);
    case 2, T_best = polyval(p2, L_smooth);
    case 3, T_best = polyval(p3, L_smooth);
    case 4, T_best = exp_model.a * exp(exp_model.b * L_smooth);
    case 5, T_best = power_model.a * L_smooth.^power_model.b;
    case 6, T_best = log_model.a * log(L_smooth) + log_model.b;
    case 7, T_best = (rat_model.p1 * L_smooth + rat_model.p2) ./ (L_smooth + rat_model.q1);
end

plot(L_smooth, T_best, 'b-', 'LineWidth', 3, 'DisplayName', sprintf('%s拟合', models{idx}));

% 标注拟合方程
switch idx
    case 1
        eq_str = sprintf('T = %.4fL + %.4f\nR² = %.5f', p1(1), p1(2), R2_best);
    case 2
        eq_str = sprintf('T = %.4fL² + %.4fL + %.4f\nR² = %.5f', p2(1), p2(2), p2(3), R2_best);
    case 3
        eq_str = sprintf('T = %.4fL³ + %.4fL² + %.4fL + %.4f\nR² = %.5f', ...
            p3(1), p3(2), p3(3), p3(4), R2_best);
    case 4
        eq_str = sprintf('T = %.4f·e^{%.4fL}\nR² = %.5f', exp_model.a, exp_model.b, R2_best);
    case 5
        eq_str = sprintf('T = %.4f·L^{%.4f}\nR² = %.5f', power_model.a, power_model.b, R2_best);
    case 6
        eq_str = sprintf('T = %.4f·ln(L) + %.4f\nR² = %.5f', log_model.a, log_model.b, R2_best);
    case 7
        eq_str = sprintf('T = (%.4fL + %.4f)/(L + %.4f)\nR² = %.5f', ...
            rat_model.p1, rat_model.p2, rat_model.q1, R2_best);
end

text(0.15, 0.85, eq_str, 'Units', 'normalized', 'FontSize', 12, ...
    'BackgroundColor', [1 1 0.8], 'EdgeColor', 'black');

xlabel('腿长 L (m)', 'FontSize', 12);
ylabel('补偿力矩 T (Nm)', 'FontSize', 12);
title(sprintf('最佳拟合: %s模型', models{idx}), 'FontSize', 13);
legend('Location', 'best', 'FontSize', 11);
grid on;

%% ========== 数据特征分析 ==========
fprintf('\n========================================\n');
fprintf('        数据特征分析\n');
fprintf('========================================\n');
fprintf('数据范围: L ∈ [%.3f, %.3f] m\n', min(L), max(L));
fprintf('力矩范围: T ∈ [%.2f, %.2f] Nm\n', min(T), max(T));
fprintf('力矩变化: %.2f Nm → %.2f Nm (变化量: %.2f Nm)\n', max(T), min(T), max(T)-min(T));
fprintf('零点穿越: T ≈ 0 出现在 L ≈ %.3f m 附近\n', L(find(T>=0, 1, 'last')));