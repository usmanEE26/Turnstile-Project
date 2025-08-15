out.active_power = out.active_power(:,1);
figure;
plot(out.tout, out.active_power)

array1 = out.active_power(:,1);
t1 = out.tout(:,1);
avg1 = mean(array1(:));


Pm = out.active_power(:, 1); % Mechanical power (W)
t = out.tout(:, 1); % Time vector (s)

% Calculate average mechanical power
Pm_avg = mean(Pm); % Average over entire simulation (W)

% Calculate total mechanical energy (integral of power over time)
energy_m = trapz(t, Pm); % Energy in Joules (negative if generated)

% Identify peak and trough values
[peak_Pm, peak_idx] = max(Pm);
[trough_Pm, trough_idx] = min(Pm);
peak_time = t(peak_idx);
trough_time = t(trough_idx);

% Assume speed data (if available, replace with actual omega_f)
load('flywheel_10s_data.mat', 'simin'); % Load time and omega_f if available
omega_f = interp1(simin.time, simin.signals.values, t); % Interpolate speed
omega_sync = pi * 50; % Synchronous speed (314.159 rad/s for 50 Hz, 2-pole)
omega_start = omega_f(1); % Initial speed
omega_end = omega_f(end); % Final speed

% Verify generation condition
is_generating = omega_start > omega_sync && Pm_avg < 0; % Condition for generation

% Estimate grid feedback (simplified, assuming 90% efficiency)
P_grid_est = 0.9 * Pm_avg; % Approximate grid power (W), adjust with actual P_s + P_r

% Plotting
figure('Position', [100 100 800 400], 'Name', 'Mechanical Power Analysis');
plot(t, Pm, 'b', 'LineWidth', 1.5);
hold on;
plot([t(1) t(end)], [0 0], 'r--', 'LineWidth', 1); % Zero line
plot(peak_time, peak_Pm, 'go', 'MarkerSize', 8, 'LineWidth', 1.5); % Peak
plot(trough_time, trough_Pm, 'ro', 'MarkerSize', 8, 'LineWidth', 1.5); % Trough
xlabel('Time (s)');
ylabel('Mechanical Power (W)');
title('Mechanical Power vs Time');
grid on;
legend('P_m', 'Zero Line', 'Peak', 'Trough', 'Location', 'best');

% Display results
fprintf('Analysis Results (as of 05:45 PM PKT, August 13, 2025):\n');
fprintf('Average Mechanical Power: %.2f W (%.2f kW)\n', Pm_avg, Pm_avg / 1000);
fprintf('Total Mechanical Energy: %.2f J (%.3f kWh)\n', energy_m, energy_m / 3600000);
fprintf('Peak Power: %.2f W at t = %.3f s\n', peak_Pm, peak_time);
fprintf('Trough Power: %.2f W at t = %.3f s\n', trough_Pm, trough_time);
fprintf('Initial Speed: %.2f rad/s, Final Speed: %.2f rad/s\n', omega_start, omega_end);
fprintf('Synchronous Speed: %.2f rad/s\n', omega_sync);
fprintf('Generation Condition Met: %d (1 = Yes, 0 = No)\n', is_generating);
fprintf('Estimated Grid Power: %.2f W (%.2f kW, 90%% of Pm_avg, adjust with actual P_s + P_r)\n', P_grid_est, P_grid_est / 1000);

% Save results
save('power_analysis_results.mat', 't', 'Pm', 'Pm_avg', 'energy_m', 'peak_Pm', 'trough_Pm', 'P_grid_est');


