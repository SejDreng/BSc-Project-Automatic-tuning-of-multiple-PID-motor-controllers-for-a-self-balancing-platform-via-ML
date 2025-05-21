function simulateRobotArm(N, L, t, init_pos, H_poisson)

    gif_filename = 'robot_arm_simulation.gif';
    save_gif = true;  % Set false to disable saving


    % Number of arm segments
    
    % Link lengths (adjustable for different configurations)
    arm_lengths = L;
    
    % Define target position within reachable range
    max_length = sum(arm_lengths);

    % target = mod(t,2*pi); % target angle in rad
    target = t;
    
    % Initial joint angles
    joint_angles = init_pos;
    prev_error = 0;

    % Simulation parameters
    max_iterations = 9000;
    tolerance = 0.0;
    dt = 0.1;
    
    % Gravity parameters (Assuming uniform gravity and link masses)
    gravity = 9.81;  % gravitational constant (m/s^2)
    link_masses = ones(1, N);  % Assume unit masses for simplicity
    

    % Fixed Plot Setup
    figure(1); clf; hold on; axis equal;
    xlim([-max_length, max_length]);
    ylim([-max_length, max_length]);
    title('Robot Arm Control Simulation');
    xlabel('X-axis'); ylabel('Y-axis');
    grid on;

    % ICO & PID Initial
    w0 = 1;
    w1_kp = ones(1, N);
    w1_kd = ones(1, N);
    w1_ki = ones(1, N);
    Kp = 0;
    Ki = 0;
    Kd = 0;
    mu = 0.00001;
    D = 0;
    integral = deg2rad(zeros(1, N));
    derivative = zeros(1, N);
    U = zeros(1, N);
    clamper = ones(1, N);
    last_iter = 0;
    X0_prev = zeros(1,N)

    % Saving values for plotting
    % Open file for writing data
    PIDfileID = fopen('pid_values.csv', 'w'); 
    ICOfileID = fopen('ICO_values.csv', 'w');
    POSfileID = fopen('pos_U_values.csv', 'w'); 
    
    PID_header = join(["Kp" + (1:N), "Ki" + (1:N), "Kd" + (1:N)], ',');
    ICO_header = join(["w1_kp" + (1:N), "w1_ki" + (1:N), "w1_kd" + (1:N)], ',');
    POS_header = join(["angle"+ (1:N), "U"+ (1:N)],',');

    fprintf(PIDfileID, '%s\n', PID_header);
    fprintf(ICOfileID, '%s\n', ICO_header);
    fprintf(POSfileID, '%s\n', POS_header);


    % Maintain a buffer of past raw_derivative values (size of buffer = filter length)
    buffer_size = length(H_poisson);
    conv_len = buffer_size + length(H_poisson) - 1;
    derivative_buffer = [];


    % Control Loop
    for iter = 1:max_iterations
   
        % Reference cascade
        % Compute cumulative joint angles
        cumulative_angles = cumsum(joint_angles);
        
        % Compute error
        error = -cumulative_angles + target;
        X0 = prev_error;  % Take scalar from previous error
        integral = (integral + error * dt .* clamper);


        %Derivative filters, sould be changed based on desired effect
        
        % Low-pass filtered derivative (Just a constant dampening in this iteration.)
        raw_derivative = (error - prev_error) / dt;
        ICO_derivative = (X0 - X0_prev) / dt;  % 1 × N
        % Update derivative buffer
        if size(derivative_buffer, 1) < buffer_size
            derivative_buffer = [ICO_derivative; derivative_buffer];
        else
            derivative_buffer = [ICO_derivative; derivative_buffer(1:end-1, :)];
        end
        
        % Compute filtered derivative
        ICO_derivative = arrayfun(@(i)conv(derivative_buffer(1:min(iter, buffer_size), i), H_poisson, 'same'), 1:N, 'UniformOutput', false);
        ICO_derivative = cellfun(@(c) c(end), ICO_derivative);

        %Kp
        X1_kp = error;
        dw1_kp = mu .* (X1_kp .* ICO_derivative); 
        w1_kp = w1_kp + dw1_kp;
        Kp = (X0 * w0) + (X1_kp .* w1_kp) ;

        %Ki
        X1_ki = error .* integral;
        dw1_ki = mu * (X1_ki .* ICO_derivative);
        w1_ki = (w1_ki + dw1_ki);
        Ki = (X0 * w0) + (X1_ki .* w1_ki);

        %Kd
        X1_kd = error .* raw_derivative;
        dw1_kd = mu * (X1_kd .* ICO_derivative);
        w1_kd = w1_kd + dw1_kd;
        Kd = (X0 * w0) + (X1_kd .* w1_kd);

        %anti windup
        is_saturated = abs(U) >= 255;
        is_winding_up = (sign(error) == sign(U));
        is_error_growing = abs(error) > abs(prev_error);
        clamper = ~(is_saturated & is_winding_up & is_error_growing);

        U = (Kp .* error + Ki .* integral + Kd .* raw_derivative);
        
        %saturation
        for i = 1:N
            if abs(U(i)) > 255
                U(i) = 255*sign(U(i));
            end
        end
        
        % Random Diturbance on every joint
        if last_iter + 100 == iter
            D = [0.3*pi 0.3*pi];
            last_iter = iter;
        else 
            D = (zeros(1, N));
        end

        
        % Write PID values to file
        fprintf(PIDfileID, ' %s\n', sprintf('%.6f,', [Kp, Ki, Kd]));
        
        % Write ICO values to file
        fprintf(ICOfileID, '%s\n', sprintf('%.6f,', [w1_kp, w1_ki, w1_kd]));
        
        % Write pos and U values to file 
        fprintf(POSfileID, ' %s\n', sprintf('%.6f,', [cumulative_angles, U]));


        pwm_signal = max(0, min(1, abs(U) / 255));
        motor_speed = pwm_signal * 0.5*pi;  
        direction = sign(U);  
        
        % Gravity disturbance (Torque due to gravity acting on each link)
        gravity_torque = zeros(1, N);
        for i = 1:N
            theta_i = cumulative_angles(i);  % sum(joint_angles(1:i))
            cos_theta_i = cos(theta_i);
            
            % Pre-compute distance from joint i to CoM of links j = i to N
            r_com = cumsum([0, arm_lengths(i+1:end)]);  % arm_lengths(i:j-1)
            r_com = r_com + arm_lengths(i:end)/2;
        
            % Vectorized torque computation for j = i:N
            torque_terms = gravity * link_masses(i:end) .* r_com * cos_theta_i;
            gravity_torque(i) = sum(torque_terms);
        end

        I = (1/3) * link_masses .* (arm_lengths .^ 2);  % Uniform rod about one end
        gravity_acc = gravity_torque ./ I;  % [rad/s^2]
        dt_grav = dt;
        gravity_displacement = -(0.5 * gravity_acc * dt_grav^2); % Multiply by 0 to turn off gravity
        new_angles = gravity_displacement + D + joint_angles + direction .* motor_speed * dt;

        X0_prev = X0;
        prev_error = error;
        
        plotRobot(new_angles, arm_lengths, N, joint_angles, gif_filename, save_gif, iter);

        joint_angles = new_angles;
        pause(0.01);

    end

    % Close file after loop ends
    fclose(PIDfileID);
    fclose(ICOfileID);
end


function coords = forwardKinematics(angles, L, N)
    coords = zeros(2, N + 1);
    for i = 1:N
        coords(:, i + 1) = coords(:, i) + [L(i) * cos(sum(angles(1:i))); L(i) * sin(sum(angles(1:i)))];
    end
end


function plotRobot(joint_angles, arm_lengths, N, prev_angles, gif_filename, save_gif, iter)
    persistent h_lines h_joints h_end fig_handle
    num_steps = 2; % Number of interpolation steps

    % Check if figure exists, if not, reinitialize
    if isempty(fig_handle) || ~isvalid(fig_handle)
        fig_handle = figure(1);
        clf; hold on; axis equal;
        xlim([-sum(arm_lengths), sum(arm_lengths)]);
        ylim([-sum(arm_lengths), sum(arm_lengths)]);
        grid on;
        title('Robot Arm Control Simulation');
        xlabel('X-axis'); ylabel('Y-axis');

        % Reset persistent variables
        h_lines = [];
        h_joints = [];
        h_end = [];
    end

    % Compute initial positions
    coords = forwardKinematics(prev_angles, arm_lengths, N);

    % Create or update plot objects
    if isempty(h_lines) || ~isvalid(h_lines)
        h_lines = plot(coords(1, :), coords(2, :), 'bo-', 'LineWidth', 2);
        h_joints = plot(coords(1, 1:end-1), coords(2, 1:end-1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        h_end = plot(coords(1, end), coords(2, end), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    end

    % Interpolate angles over multiple steps for smooth rotation
    for step = 1:num_steps
        interp_angles = prev_angles + (joint_angles - prev_angles) * (step / num_steps);
        interp_coords = forwardKinematics(interp_angles, arm_lengths, N);

        % Update plot objects
        set(h_lines, 'XData', interp_coords(1, :), 'YData', interp_coords(2, :));
        set(h_joints, 'XData', interp_coords(1, 1:end-1), 'YData', interp_coords(2, 1:end-1));
        set(h_end, 'XData', interp_coords(1, end), 'YData', interp_coords(2, end));

        drawnow;
        pause(0.001); % Small delay for smooth animation
        if save_gif
            frame = getframe(gcf);
            img = frame2im(frame);
            [imind, cm] = rgb2ind(img, 256);
        
            if iter == 1 && step == 1
                imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.001);
            else
                imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.001);
            end
        end
    end
end

function h = poisson_bandpass_1d(fil_len, low_lambda, high_lambda)
% POISSON_BANDPASS_1D Creates a 1D Poisson-like band-pass filter
%   N: Filter length (odd number recommended)
%   low_lambda: Controls high-pass decay
%   high_lambda: Controls low-pass decay

%   Returns:
%   h: 1D filter coefficients

    n = 0:fil_len-1;  % or n = 0:100
    lp = exp(-n / high_lambda);
    hp = exp(-n / low_lambda);
    h = hp - lp;
    h = h / sum(abs(h));

end

% Generate filter
fil_len = 101;                       % Filter length
low_lambda = 40.0;               % Controls high-pass decay
high_lambda = 2;               % Controls low-pass decay
H = poisson_bandpass_1d(fil_len, low_lambda, high_lambda);

N = 2;

L = [10 10];

t = 0*pi;

init_pos = [0*pi 0];

% init_pos = zeros(1, N);

figure(2)
plot(H); title('Poisson Band-Pass Filter Coefficients');

% Display coefficients in a C++ array format
fprintf('const int FILTER_LEN = %d;\n', fil_len);
fprintf('const float H_poisson[FILTER_LEN] = {\n    ');
for i = 1:fil_len
    fprintf('%.8ff', H(i));
    if i < fil_len
        fprintf(', ');
        if mod(i, 5) == 0 % Newline every 5 coefficients for readability
            fprintf('\n    ');
        end
    end
end
fprintf('\n};\n');


simulateRobotArm(N,L,t,init_pos,H)

