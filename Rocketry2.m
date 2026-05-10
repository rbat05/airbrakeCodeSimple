% ============================================================
% EKF CSV Plotter (matches original live serial layout)
% Serial/Csv format expected per line:
% millis, baro_h, velocity, ekf_h, ekf_v
% Matches ESP32 Serial.printf exactly
% ============================================================

% --- Configuration ---
csvFile = "traceout.csv";    % path to CSV file that uses same line format
simulateRealtime = false; % true -> pause following CSV millis intervals

WINDOW_SEC = 60;   % show last 60 seconds
t0 = NaN;
max_h_seen = 0;    % track peak height to auto-scale
last_ms = NaN;     % track previous millis to ensure monotonicity

% --- Original live serial code commented out ---
% s = serialport("/dev/ttyUSB0", 115200); % Linux version
% configureTerminator(s, "LF");
% flush(s);

% --- Prepare figure (same layout as original) ---
fig = figure('Name', 'EKF Live Monitor (CSV)', 'NumberTitle', 'off', ...
             'Position', [100 100 1000 650]);

% 1. HEIGHT — baro vs EKF
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
ax1.YLim = [-10 500];

% 2. VELOCITY — IMU vs EKF
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
ax2.YLim = [-50 200];

% --- Open CSV and process line-by-line using same parsing semantics ---
fid = fopen(csvFile, 'r');
if fid == -1
    error("Unable to open CSV file: %s", csvFile);
end

try
    while ishandle(fig)
        tline = fgetl(fid);
        if tline == -1
            break; % EOF
        end

        % Parse line the same way as serial: trim, split by comma, convert
        parts = split(strtrim(tline), ",");
        data = str2double(parts);

        % Need exactly 5 numeric columns; skip malformed or header
        if length(data) < 5 || any(isnan(data))
            continue; % skip nonnumeric/header/malformed lines
        end

        % Unpack — matches ESP32 printf column order
        t_ms    = data(1);   % millis()
        baro_h  = data(2);   % barometer_raw (m)
        vel_imu = data(3);   % velocity        (m/s, IMU integrated)
        ekf_h   = data(4);   % ekf.x[0]        (m)
        ekf_v   = data(5);   % ekf.x[1]        (m/s)

        % Ensure millis is numeric and nonnegative
        if ~isfinite(t_ms) || t_ms < 0
            continue;
        end

        % Set t0 on first valid packet only
        if isnan(t0)
            t0 = t_ms;
            last_ms = t_ms;
            disp('First packet received — plotting started (from CSV)');
        else
            % If this row is out-of-order (earlier millis), skip it
            if t_ms < last_ms
                % skip to avoid negative/going-back-in-time
                continue;
            end
            last_ms = t_ms;
        end

        % Compute time in seconds relative to t0
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

        % Simulate realtime timing if requested: pause using difference to next line's millis
        if simulateRealtime
            curPos = ftell(fid);
            nextLine = fgetl(fid);
            if nextLine ~= -1
                nparts = split(strtrim(nextLine), ",");
                ndata = str2double(nparts);
                if length(ndata) >= 1 && ~isnan(ndata(1))
                    next_ms = ndata(1);
                    if next_ms >= t_ms
                        dt = (next_ms - t_ms) / 1000;
                        if dt > 0
                            pause(dt);
                        else
                            pause(0.001);
                        end
                    else
                        pause(0.001);
                    end
                else
                    pause(0.001);
                end
                fseek(fid, curPos, 'bof'); % rewind
            else
                pause(0.01);
            end
        end
    end

    disp('Finished plotting CSV data.');

catch ME
    fclose(fid);
    rethrow(ME);
end

fclose(fid);

% --- Clean up (originally cleared serial) ---
% clear s;
% disp('Serial port closed.');
