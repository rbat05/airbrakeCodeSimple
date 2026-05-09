% ============================================================
% EKF Live Plotter
% Serial format: millis, baro_h, velocity, ekf_h, ekf_v
% Matches ESP32 Serial.printf exactly
% ============================================================

% --- Setup serial ---
s = serialport("COM5", 115200);
configureTerminator(s, "LF");
flush(s);

% --- Figure ---
fig = figure('Name', 'EKF Live Monitor', 'NumberTitle', 'off', ...
             'Position', [100 100 1000 650]);

% =========================
% 1. HEIGHT — baro vs EKF
% =========================
ax1 = subplot(2,1,1);
h_baro = animatedline(ax1, 'Color', [0.8 0.3 0.3], ...
                      'LineWidth', 1.0, 'DisplayName', 'Raw Barometer');
h_ekf  = animatedline(ax1, 'Color', [0.2 0.6 1.0], ...
                      'LineWidth', 2.0, 'DisplayName', 'EKF Height');
title(ax1, 'Height — Raw Barometer vs EKF Estimate');
xlabel(ax1, 'Time (s)');
ylabel(ax1, 'Height (m)');
legend(ax1, 'Location', 'northwest');
grid(ax1, 'on');
ax1.YLim = [-10 500];   % expected range for ~450m apogee rocket

% =========================
% 2. VELOCITY — IMU vs EKF
% =========================
ax2 = subplot(2,1,2);
v_imu = animatedline(ax2, 'Color', [0.3 0.8 0.3], ...
                     'LineWidth', 1.0, 'DisplayName', 'IMU Integrated');
v_ekf = animatedline(ax2, 'Color', [0.2 0.6 1.0], ...
                     'LineWidth', 2.0, 'DisplayName', 'EKF Velocity');
title(ax2, 'Velocity — IMU Integrated vs EKF Estimate');
xlabel(ax2, 'Time (s)');
ylabel(ax2, 'Velocity (m/s)');
legend(ax2, 'Location', 'northwest');
grid(ax2, 'on');
ax2.YLim = [-50 200];   % expected velocity range

WINDOW_SEC = 60;   % show last 60 seconds
t0 = NaN;
max_h_seen = 0;    % track peak height to auto-scale

% --- Main loop ---
try
    while ishandle(fig)

        line = readline(s);
        data = str2double(split(strtrim(line), ","));

        % Skip malformed lines — need exactly 5 columns
        if length(data) < 5 || any(isnan(data))
            continue;
        end

        % Unpack — matches ESP32 printf column order
        t_ms    = data(1);   % millis()
        baro_h  = data(2);   % barometer_raw  (m)
        vel_imu = data(3);   % velocity       (m/s, IMU integrated)
        ekf_h   = data(4);   % ekf.x[0]       (m)
        ekf_v   = data(5);   % ekf.x[1]       (m/s)

        % Zero time on first valid packet
        if isnan(t0)
            t0 = t_ms;
            disp('First packet received — plotting started');
        end
        t = (t_ms - t0) / 1000.0;

        % --- Add points ---
        addpoints(h_baro, t, baro_h);
        addpoints(h_ekf,  t, ekf_h);
        addpoints(v_imu,  t, vel_imu);
        addpoints(v_ekf,  t, ekf_v);

        % Auto-scale height axis if rocket goes higher than default
        if ekf_h > max_h_seen
            max_h_seen = ekf_h;
            if max_h_seen > 400
                ax1.YLim = [-10, max_h_seen * 1.1];
            end
        end

        % Rolling x-axis window
        if t > WINDOW_SEC
            xlim(ax1, [t - WINDOW_SEC, t]);
            xlim(ax2, [t - WINDOW_SEC, t]);
        end

        drawnow limitrate;
    end

catch ME
    if contains(ME.message, 'terminated by user') || ...
       contains(ME.identifier, 'asyncio')         || ...
       contains(ME.identifier, 'seriallib')
        disp('Plotting stopped by user.');
    else
        disp('Unexpected error:');
        disp(ME.message);
    end
end

clear s;
disp('Serial port closed.');