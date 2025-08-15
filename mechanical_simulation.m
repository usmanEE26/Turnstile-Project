%% Simulation 1
% % spring_flywheel_noDFIG_FIXED.m
% % ---------------------------------------------------------------
% % Low-RPM turnstile -> gear -> torsion spring -> one-way clutch -> flywheel
% % No generator torque. Visualize pure overspeed pulses + energy.
% % ---------------------------------------------------------------
% 
% clear; clc;
% 
% %% ------------ BASIC PARAMETERS ---------------------------------
% Jf        = 0.2;            % flywheel inertia  [kg·m^2]
% k_spring  = 8e3;            % spring stiffness  [Nm/rad]
% tau_eng   = 1.5e3;          % clutch engage threshold [Nm]
% G         = 100;            % gear ratio (torque ×G ; speed ÷G)
% 
% f_grid     = 50;            % grid electrical frequency [Hz]
% poles      = 2;             % 2-pole machine -> mech sync = 2*pi*f
% omega_sync = pi*f_grid;   % = 314.159... rad/s for 50 Hz, 2-pole
% C_damp     = 0.1;           % Nm·s/rad  (tiny viscous drag)
% 
% % Optional: small hysteresis to avoid chatter
% use_hyst = true;
% tau_rel  = 0.95*tau_eng;    % release threshold (only if use_hyst)
% 
% %% ------------ HUMAN INPUT PROFILE (speed-driven) ---------------
% t_end = 240;  dt = 1e-3;   t = 0:dt:t_end;
% 
% % Push 0.5 s every 3 s at 2 rad/s on the turnstile shaft (LOW-speed side)
% omega_turn = zeros(size(t));
% push_mask  = mod(t,3) < 0.5;      % <-- truly every 3 s, 0.5 s on
% omega_turn(push_mask) = 2.0;      % prescribed LOW-speed input
% 
% %% ------------ PRE-ALLOCATE STATES ------------------------------
% omega_f      = omega_sync * ones(size(t));  % start at synchronous
% theta_s      = zeros(size(t));
% clutch_state = zeros(size(t));             % 0 = free, 1 = engaged
% 
% % Diagnostics: energy accounting w.r.t. sync
% E_spring = zeros(size(t));                 % 0.5*k*theta_s^2
% E_fly    = zeros(size(t));                 % 0.5*Jf*(omega_f-omega_sync)^2
% E_spring(1) = 0.5*k_spring*theta_s(1)^2;
% E_fly(1)    = 0.5*Jf*(omega_f(1)-omega_sync)^2;
% 
% %% ------------ MAIN LOOP ----------------------------------------
% engaged = false;
% for k = 1:numel(t)-1
% 
%     % Gear-side kinematics
%     omega_in =  omega_turn(k)/G;     % HIGH-speed shaft input
%     tau_s    =  k_spring*theta_s(k); % spring torque
% 
%     % Clutch logic (with optional hysteresis)
%     if use_hyst
%         if ~engaged && (tau_s >= tau_eng)
%             engaged = true;
%         elseif engaged && (tau_s <= tau_rel)
%             engaged = false;
%         end
%     else
%         engaged = (tau_s >= tau_eng);
%     end
%     clutch_state(k) = engaged;
% 
%     % Relative twist rate of the spring
%     % - Disengaged: omega_out = 0 -> spring winds at omega_in
%     % - Engaged:    omega_out = omega_f -> spring discharges into flywheel
%     omega_out = engaged * omega_f(k);
%     dtheta_s  = omega_in - omega_out;
% 
%     % Torque into flywheel via clutch (only when engaged)
%     tau_clutch = engaged * tau_s;
% 
%     % Flywheel dynamics (spring + tiny viscous pull toward sync)
%     tau_drag  = -C_damp * (omega_f(k) - omega_sync);
%     domega_f  = (tau_clutch + tau_drag)/Jf;
% 
%     % Integrate states
%     theta_s(k+1) = theta_s(k) + dtheta_s*dt;
%     omega_f(k+1) = omega_f(k) + domega_f*dt;
% 
%     % Energies (post-update)
%     E_spring(k+1) = 0.5*k_spring*theta_s(k+1)^2;
%     E_fly(k+1)    = 0.5*Jf*(omega_f(k+1)-omega_sync)^2;
% end
% clutch_state(end) = clutch_state(end-1);
% 
% %% ------------ PLOTS -------------------------------------------
% figure('Position',[50 50 760 800],'Name','Flywheel with Pure Spring Pulses');
% 
% subplot(4,1,1)
% plot(t,omega_f,'b', t,omega_sync+zeros(size(t)),'--r','LineWidth',1.3)
% ylabel('\omega_f (rad/s)')
% title('Flywheel Speed   (red = synchronous)'); grid on
% 
% subplot(4,1,2)
% plot(t,k_spring*theta_s,'LineWidth',1.3)
% ylabel('\tau_{spring} (Nm)')
% title('Spring Torque'); grid on
% 
% subplot(4,1,3)
% stairs(t,clutch_state,'LineWidth',1.3); ylim([-0.2 1.2])
% ylabel('Clutch'); xlabel('Time (s)')
% title('0 = free,   1 = engaged'); grid on
% 
% subplot(4,1,4)
% plot(t,E_spring,'LineWidth',1.3); hold on
% plot(t,E_fly,'LineWidth',1.3)
% ylabel('Energy (J)'); xlabel('Time (s)'); grid on
% legend('Spring','Flywheel (vs sync)','Location','best')
% title('Energy Transfer Diagnostics')
% 
% %% ------------ OVERSPEED PEAK EXTRACTION ------------------------
% % Step 1 — define overspeed signal relative to synchronous speed
% delta = omega_f - omega_sync;   % positive means overspeed
% 
% % Step 2 — find peaks where delta > 0
% % - MinPeakHeight = 0 ensures we only accept overspeed (above sync)
% % - MinPeakProminence filters out tiny wiggles; tune as needed
% % - MinPeakDistance avoids picking peaks from the same pulse too close together
% [pks, locs, widths, prom] = findpeaks( ...
%     delta, t, ...
%     'MinPeakHeight', 0, ...
%     'MinPeakProminence', 1, ...     % <-- tune if you get too many/too few peaks
%     'MinPeakDistance', 0.5, ...     % <-- seconds between peaks (tune)
%     'WidthReference','halfprom');
% 
% if isempty(pks)
%     warning('No overspeed peaks found (omega_f never exceeded omega_sync).');
% else
%     % Step 3 — take the FIRST overspeed peak
%     t_pk      = locs(1);            % time of peak
%     dOmega_pk = pks(1);             % overspeed above sync at the peak
%     omega_pk  = omega_sync + dOmega_pk;
%     w_pk      = widths(1);          % half-prominence width (in seconds)
%     prom_pk   = prom(1);            % prominence (rad/s)
% 
%     % Step 4 — define a simple window around the peak using the width
%     tL = t_pk - w_pk/2;
%     tR = t_pk + w_pk/2;
% 
%     % Clip to available time range
%     tL = max(tL, t(1));
%     tR = min(tR, t(end));
% 
%     % Extract the segment around the peak (for saving/analysis)
%     seg_idx  = (t >= tL) & (t <= tR);
%     peak_t   = t(seg_idx);
%     peak_w   = omega_f(seg_idx);
% 
%     % Print a concise summary
%     fprintf('First overspeed peak:\n');
%     fprintf('  t = %.4f s\n', t_pk);
%     fprintf('  omega_peak = %.6f rad/s\n', omega_pk);
%     fprintf('  overspeed above sync = %.6f rad/s\n', dOmega_pk);
%     fprintf('  prominence = %.6f rad/s\n', prom_pk);
%     fprintf('  half-prominence width = %.6f s (window [%.4f, %.4f])\n', w_pk, tL, tR);
% 
%     % Optional: put the extracted segment into a struct
%     overspeed_peak = struct( ...
%         't_peak', t_pk, ...
%         'omega_peak', omega_pk, ...
%         'overspeed_peak', dOmega_pk, ...
%         'prominence', prom_pk, ...
%         'halfprom_width_s', w_pk, ...
%         't_window', [tL tR], ...
%         'segment_time', peak_t, ...
%         'segment_omega', peak_w);
% 
%     % --------- Annotate your existing flywheel-speed subplot ----------
%     % If you already created the figure above, this will add markers on it.
%     figure(gcf);                        % use current figure
%     subplot(4,1,1); hold on;
% 
%     % Mark the peak point
%     plot(t_pk, omega_pk, 'ro', 'MarkerSize', 6, 'LineWidth', 1.4);
% 
%     % Vertical line at peak
%     xline(t_pk, '--r', 'LineWidth', 1);
% 
%     % Lightly shade the half-prominence window for context
%     yl = ylim;
%     patch([tL tR tR tL], [yl(1) yl(1) yl(2) yl(2)], ...
%           'r', 'FaceAlpha', 0.06, 'EdgeColor', 'none');
% 
%     % Label
%     txt = sprintf('\\omega_{pk}=%.2f rad/s (\\Delta=%.2f)', omega_pk, dOmega_pk);
%     text(t_pk, omega_pk, ['  ' txt], 'VerticalAlignment','bottom');
% 
%     hold off;
% end
% 
% %% --- Extract ONE overspeed peak (full trace), re-zero time, and plot ---
% delta = omega_f - omega_sync;   % overspeed signal (>0 above sync)
% 
% % Find candidate peaks of delta
% [pks, locs] = findpeaks(delta, t, ...
%     'MinPeakHeight', 0, ...
%     'MinPeakProminence', 1, ...
%     'MinPeakDistance', 0.5);
% 
% if isempty(pks)
%     overspeed_trace = [];
%     overspeed_time  = [];
%     overspeed_rot   = [];
%     warning('No overspeed segment found (omega_f never exceeded omega_sync).');
% else
%     % Choose the LARGEST overspeed event (use idx=1 for "first")
%     [~, idx] = max(pks);
%     t_pk = locs(idx);
% 
%     % Peak index in sampled arrays
%     [~, k_pk] = min(abs(t - t_pk));
%     dOmega_pk = delta(k_pk);
%     omega_pk  = omega_sync + dOmega_pk;
% 
%     % Bound the peak by where delta is ~0 (avoid merging events)
%     eps_abs = 0.02;                 % absolute threshold [rad/s]
%     eps_rel = 0.01 * dOmega_pk;     % 1% of peak overspeed
%     eps_cut = max(eps_abs, eps_rel);
% 
%     % expand left
%     iL = k_pk;
%     while iL > 1 && delta(iL) > eps_cut
%         iL = iL - 1;
%     end
%     % expand right
%     iR = k_pk;
%     while iR < numel(t) && delta(iR) > eps_cut
%         iR = iR + 1;
%     end
% 
%     % Build full trace with time starting at 0
%     overspeed_time = t(iL:iR) - t(iL);
%     overspeed_rot  = omega_f(iL:iR);
%     overspeed_trace = [overspeed_time(:), overspeed_rot(:)];
% 
%     % Peak time in the zeroed frame
%     t_peak0 = t(k_pk) - t(iL);
%     overspeed_peak = [t_peak0, omega_pk];   % [time_from_start, rad/s]
% 
%     % ---- Plot ONLY this peak in a separate figure ----
%     figure('Name','Single Overspeed Peak','Position',[80 80 760 420]);
%     plot(overspeed_time, overspeed_rot, 'LineWidth', 1.4); hold on
%     yline(omega_sync, '--r', 'LineWidth', 1.2);
%     plot(t_peak0, omega_pk, 'ro', 'MarkerSize', 6, 'LineWidth', 1.2);
%     xlabel('Time from start of peak (s)'); ylabel('\omega_f (rad/s)');
%     title(sprintf('Overspeed Segment: t_{pk}=%.4f s, \\omega_{pk}=%.2f rad/s', ...
%                   t_peak0, omega_pk));
%     grid on; hold off
% end
% 
% 


% spring_flywheel_noDFIG_FIXED.m
% ---------------------------------------------------------------
% Low-RPM turnstile -> gear -> torsion spring -> one-way clutch -> flywheel
% No generator torque. Extract and save a single overspeed peak for Simulink.
% ---------------------------------------------------------------

%% Simulation 2
clear; clc;

%% ------------ BASIC PARAMETERS ---------------------------------
Jf        = 0.2;            % flywheel inertia  [kg·m^2]
k_spring  = 8e3;            % spring stiffness  [Nm/rad]
tau_eng   = 1.5e3;          % clutch engage threshold [Nm]
G         = 100;            % gear ratio (torque ×G ; speed ÷G)

f_grid     = 50;            % grid electrical frequency [Hz]
poles      = 2;             % 2-pole machine -> mech sync = 2*pi*f
omega_sync = pi*f_grid;     % = 314.159... rad/s for 50 Hz, 2-pole
C_damp     = 0.1;           % Nm·s/rad  (tiny viscous drag)

% Optional: small hysteresis to avoid chatter
use_hyst = true;
tau_rel  = 0.95*tau_eng;    % release threshold (only if use_hyst)

%% ------------ HUMAN INPUT PROFILE (speed-driven) ---------------
t_end = 240;  dt = 1e-3;   t = 0:dt:t_end;

% Push 0.5 s every 3 s at 2 rad/s on the turnstile shaft (LOW-speed side)
omega_turn = zeros(size(t));
push_mask  = mod(t,3) < 0.5;      % truly every 3 s, 0.5 s on
omega_turn(push_mask) = 2.0;      % prescribed LOW-speed input

%% ------------ PRE-ALLOCATE STATES ------------------------------
omega_f      = omega_sync * ones(size(t));  % start at synchronous
theta_s      = zeros(size(t));
clutch_state = zeros(size(t));             % 0 = free, 1 = engaged

% Diagnostics: energy accounting w.r.t. sync
E_spring = zeros(size(t));                 % 0.5*k*theta_s^2
E_fly    = zeros(size(t));                 % 0.5*Jf*(omega_f-omega_sync)^2
E_spring(1) = 0.5*k_spring*theta_s(1)^2;
E_fly(1)    = 0.5*Jf*(omega_f(1)-omega_sync)^2;

%% ------------ MAIN LOOP ----------------------------------------
engaged = false;
for k = 1:numel(t)-1

    % Gear-side kinematics
    omega_in =  omega_turn(k)/G;     % HIGH-speed shaft input
    tau_s    =  k_spring*theta_s(k); % spring torque

    % Clutch logic (with optional hysteresis)
    if use_hyst
        if ~engaged && (tau_s >= tau_eng)
            engaged = true;
        elseif engaged && (tau_s <= tau_rel)
            engaged = false;
        end
    else
        engaged = (tau_s >= tau_eng);
    end
    clutch_state(k) = engaged;

    % Relative twist rate of the spring
    omega_out = engaged * omega_f(k);
    dtheta_s  = omega_in - omega_out;

    % Torque into flywheel via clutch (only when engaged)
    tau_clutch = engaged * tau_s;

    % Flywheel dynamics (spring + tiny viscous pull toward sync)
    tau_drag  = -C_damp * (omega_f(k) - omega_sync);
    domega_f  = (tau_clutch + tau_drag)/Jf;

    % Integrate states
    theta_s(k+1) = theta_s(k) + dtheta_s*dt;
    omega_f(k+1) = omega_f(k) + domega_f*dt;

    % Energies (post-update)
    E_spring(k+1) = 0.5*k_spring*theta_s(k+1)^2;
    E_fly(k+1)    = 0.5*Jf*(omega_f(k+1)-omega_sync)^2;
end
clutch_state(end) = clutch_state(end-1);

%% --- Extract ONE overspeed peak (full trace), re-zero time ---
delta = omega_f - omega_sync;   % overspeed signal (>0 above sync)

% Find candidate peaks of delta
[pks, locs] = findpeaks(delta, t, ...
    'MinPeakHeight', 0, ...
    'MinPeakProminence', 1, ...
    'MinPeakDistance', 0.5);

if isempty(pks)
    overspeed_trace = [];
    overspeed_time  = [];
    overspeed_rot   = [];
    warning('No overspeed segment found (omega_f never exceeded omega_sync).');
else
    % Choose the LARGEST overspeed event
    [~, idx] = max(pks);
    t_pk = locs(idx);

    % Peak index in sampled arrays
    [~, k_pk] = min(abs(t - t_pk));
    dOmega_pk = delta(k_pk);
    omega_pk  = omega_sync + dOmega_pk;

    % Bound the peak by where delta is ~0 (avoid merging events)
    eps_abs = 0.02;                 % absolute threshold [rad/s]
    eps_rel = 0.01 * dOmega_pk;     % 1% of peak overspeed
    eps_cut = max(eps_abs, eps_rel);

    % Expand left
    iL = k_pk;
    while iL > 1 && delta(iL) > eps_cut
        iL = iL - 1;
    end
    % Expand right
    iR = k_pk;
    while iR < numel(t) && delta(iR) > eps_cut
        iR = iR + 1;
    end

    % Build full trace with time starting at 0
    overspeed_time = t(iL:iR) - t(iL);
    overspeed_rot  = omega_f(iL:iR);

    % Optional: plot for verification (comment out if not needed)
    figure('Name','Single Overspeed Peak','Position',[80 80 760 420]);
    plot(overspeed_time, overspeed_rot, 'LineWidth', 1.4); hold on
    yline(omega_sync, '--r', 'LineWidth', 1.2);
    plot(overspeed_time(k_pk-iL+1), omega_pk, 'ro', 'MarkerSize', 6, 'LineWidth', 1.2);
    xlabel('Time from start of peak (s)'); ylabel('\omega_f (rad/s)');
    title(sprintf('Overspeed Segment: t_{pk}=%.4f s, \\omega_{pk}=%.2f rad/s', ...
                  overspeed_time(k_pk-iL+1), omega_pk));
    grid on; hold off
end

%% ------------ SAVE DATA FOR SIMULINK --------------------------
if ~isempty(overspeed_time)
    simin.time = overspeed_time';           % Time vector as a column (zeroed)
    simin.signals.values = overspeed_rot';  % Flywheel speed as a column
    simin.signals.dimensions = 1;           % Number of signals
    simin.signals.label = 'omega_f';        % Optional label
    save('flywheel_peak_data.mat', 'simin');
else
    warning('No overspeed peak data to save. Check parameters or simulation.');
end

% Remove plots and other diagnostics if not needed for Simulink use
% (Comment out or remove the plotting sections above if desired)

%% Simulation 3
% spring_flywheel_noDFIG_FIXED.m
% ---------------------------------------------------------------
% Low-RPM turnstile -> gear -> torsion spring -> one-way clutch -> flywheel
% No generator torque. Save full omega_f for Simulink integration.
% ---------------------------------------------------------------

% clear; clc;
% 
% %% ------------ BASIC PARAMETERS ---------------------------------
% Jf        = 0.2;            % flywheel inertia  [kg·m^2]
% k_spring  = 8e3;            % spring stiffness  [Nm/rad]
% tau_eng   = 1.5e3;          % clutch engage threshold [Nm]
% G         = 100;            % gear ratio (torque ×G ; speed ÷G)
% 
% f_grid     = 50;            % grid electrical frequency [Hz]
% poles      = 2;             % 2-pole machine -> mech sync = 2*pi*f
% omega_sync = pi*f_grid;     % = 314.159... rad/s for 50 Hz, 2-pole
% C_damp     = 0.1;           % Nm·s/rad  (tiny viscous drag)
% 
% % Optional: small hysteresis to avoid chatter
% use_hyst = true;
% tau_rel  = 0.95*tau_eng;    % release threshold (only if use_hyst)
% 
% %% ------------ HUMAN INPUT PROFILE (speed-driven) ---------------
% t_end = 30;  dt = 1e-3;   t = 0:dt:t_end;
% 
% % Push 0.5 s every 3 s at 2 rad/s on the turnstile shaft (LOW-speed side)
% omega_turn = zeros(size(t));
% push_mask  = mod(t,0.25) < 0.5;      % truly every 3 s, 0.5 s on
% omega_turn(push_mask) = 2.0;      % prescribed LOW-speed input
% 
% %% ------------ PRE-ALLOCATE STATES ------------------------------
% omega_f      = omega_sync * ones(size(t));  % start at synchronous
% theta_s      = zeros(size(t));
% clutch_state = zeros(size(t));             % 0 = free, 1 = engaged
% 
% % Diagnostics: energy accounting w.r.t. sync
% E_spring = zeros(size(t));                 % 0.5*k*theta_s^2
% E_fly    = zeros(size(t));                 % 0.5*Jf*(omega_f-omega_sync)^2
% E_spring(1) = 0.5*k_spring*theta_s(1)^2;
% E_fly(1)    = 0.5*Jf*(omega_f(1)-omega_sync)^2;
% 
% %% ------------ MAIN LOOP ----------------------------------------
% engaged = false;
% for k = 1:numel(t)-1
% 
%     % Gear-side kinematics
%     omega_in =  omega_turn(k)/G;     % HIGH-speed shaft input
%     tau_s    =  k_spring*theta_s(k); % spring torque
% 
%     % Clutch logic (with optional hysteresis)
%     if use_hyst
%         if ~engaged && (tau_s >= tau_eng)
%             engaged = true;
%         elseif engaged && (tau_s <= tau_rel)
%             engaged = false;
%         end
%     else
%         engaged = (tau_s >= tau_eng);
%     end
%     clutch_state(k) = engaged;
% 
%     % Relative twist rate of the spring
%     omega_out = engaged * omega_f(k);
%     dtheta_s  = omega_in - omega_out;
% 
%     % Torque into flywheel via clutch (only when engaged)
%     tau_clutch = engaged * tau_s;
% 
%     % Flywheel dynamics (spring + tiny viscous pull toward sync)
%     tau_drag  = -C_damp * (omega_f(k) - omega_sync);
%     domega_f  = (tau_clutch + tau_drag)/Jf;
% 
%     % Integrate states
%     theta_s(k+1) = theta_s(k) + dtheta_s*dt;
%     omega_f(k+1) = omega_f(k) + domega_f*dt;
% 
%     % Energies (post-update)
%     E_spring(k+1) = 0.5*k_spring*theta_s(k+1)^2;
%     E_fly(k+1)    = 0.5*Jf*(omega_f(k+1)-omega_sync)^2;
% end
% clutch_state(end) = clutch_state(end-1);
% 
% %% ------------ PLOTS (Optional for Verification) ----------------
% figure('Position',[50 50 760 800],'Name','Flywheel with Pure Spring Pulses');
% 
% subplot(4,1,1)
% plot(t,omega_f,'b', t,omega_sync+zeros(size(t)),'--r','LineWidth',1.3)
% ylabel('\omega_f (rad/s)')
% title('Flywheel Speed   (red = synchronous)'); grid on
% 
% subplot(4,1,2)
% plot(t,k_spring*theta_s,'LineWidth',1.3)
% ylabel('\tau_{spring} (Nm)')
% title('Spring Torque'); grid on
% 
% subplot(4,1,3)
% stairs(t,clutch_state,'LineWidth',1.3); ylim([-0.2 1.2])
% ylabel('Clutch'); xlabel('Time (s)')
% title('0 = free,   1 = engaged'); grid on
% 
% subplot(4,1,4)
% plot(t,E_spring,'LineWidth',1.3); hold on
% plot(t,E_fly,'LineWidth',1.3)
% ylabel('Energy (J)'); xlabel('Time (s)'); grid on
% legend('Spring','Flywheel (vs sync)','Location','best')
% title('Energy Transfer Diagnostics')
% 
% %% ------------ SAVE FULL omega_f DATA FOR SIMULINK -------------
% simin.time = t';           % Full time vector as a column
% simin.signals.values = omega_f'; % Full flywheel speed as a column
% simin.signals.dimensions = 1;    % Number of signals
% simin.signals.label = 'omega_f'; % Optional label
% save('flywheel_full_data.mat', 'simin');
% 
% %% Simulation 4
