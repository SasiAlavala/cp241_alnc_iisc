% ============================================================
% Creating ROS2 Node
% ============================================================

%setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp");
setenv("ROS_DOMAIN_ID","30");
node = ros2node("/matlab_pid_controller");

% ============================================================
% Publisher & Subscriber
% ============================================================

% Each new MOCAP pose records its arrival time, so an old pose can be detected
setappdata(groot, 'pid_last_pose_time', []);
poseSub = ros2subscriber(node, ...
    "/phasespace/pose", ...
    "geometry_msgs/Pose2D", ...
    @(msg) setappdata(groot, 'pid_last_pose_time', tic));

cmdPub = ros2publisher(node, ...
    "/HWTB3_10/cmd_vel", ...
    "geometry_msgs/Twist");

cmdMsg = ros2message(cmdPub);

% ============================================================
% Target Pose (must be inside the MOCAP area)
% ============================================================

xd = 5.02;
yd = 1.78;
theta_d = 0.0;

% ============================================================
% PID Errors
% ============================================================

ep_prev = 0;
ei_p = 0;

ea_prev = 0;
ei_a = 0;

first_loop = true;      % no previous error yet: skip the derivative once
v_prev = 0;
pose_lost = false;

% ============================================================
% PID Gains - Position (same as simulation)
% ============================================================

kp_p = 0.30;
kd_p = 0.05;
ki_p = 0.01;

% ============================================================
% PID Gains - Heading (same as simulation)
% ============================================================

kp_a = 1.00;
kd_a = 0.10;
ki_a = 0.01;

% ============================================================
% Limits
% ============================================================

V_MAX = 0.2;            % m/s
W_MAX = 0.2;            % rad/s
tol = 0.05;             % m, stop this close to the goal (2 cm in simulation; real motors need more)
pose_timeout = 0.5;     % s, stop if no new MOCAP pose for this long
max_accel = 0.5;        % m/s^2, ramp v so the robot does not jerk

% ============================================================
% Timing
% ============================================================

timer = tic;            % time since the previous loop (dt)
t_start = tic;          % time since start (trajectory)
rate = robotics.Rate(10);   % 10 Hz, same as simulation

% ============================================================
% TRAJECTORY VARIABLES
% ============================================================

% Preallocate some space
trajectory_x = [];
trajectory_y = [];
trajectory_t = [];

start_x = [];
start_y = [];

% ============================================================
% LIVE FIGURE
% ============================================================

figure(1);
clf;

hold on;
grid on;
axis equal;

xlabel('X Position');
ylabel('Y Position');
title('Robot Trajectory');

% Goal
goalPlot = plot(xd, yd, 'rx', ...
    'MarkerSize', 12, ...
    'LineWidth', 3);

% Trajectory line
trajectoryPlot = plot(nan, nan, 'b-', ...
    'LineWidth', 2);

% Current robot position
robotPlot = plot(nan, nan, 'ko', ...
    'MarkerSize', 8, ...
    'MarkerFaceColor', [0.8 0.95 1.0]);

% Start point
startPlot = plot(nan, nan, 'go', ...
    'MarkerSize', 10, ...
    'LineWidth', 2);

legend('Goal', 'Trajectory', 'Robot', 'Start');

drawnow;


% ============================================================
% MAIN CONTROL LOOP
% ============================================================
disp("Starting controller! (Ctrl+C stops this script; then run stop.m to stop the robot)")
while true

    % --------------------------------------------------------
    % Get latest PhaseSpace pose
    % --------------------------------------------------------
    poseMsg = poseSub.LatestMessage;
    last_pose_time = getappdata(groot, 'pid_last_pose_time');

    if isempty(poseMsg) || isempty(last_pose_time)
        waitfor(rate);
        continue;
    end

    xc1 = double(poseMsg.x);
    yc1 = double(poseMsg.y);

    theta_c = double(poseMsg.theta);

    % --------------------------------------------------------
    % Pose check: stop while the pose is old or invalid
    % (markers hidden from the cameras), resume when it is back
    % --------------------------------------------------------

    if toc(last_pose_time) > pose_timeout || ~all(isfinite([xc1, yc1, theta_c]))
        if ~pose_lost
            fprintf('No valid MOCAP pose, stopping\n');
            pose_lost = true;
        end
        cmdMsg.linear.x = 0;
        cmdMsg.angular.z = 0;
        send(cmdPub, cmdMsg);
        v_prev = 0;
        first_loop = true;
        timer = tic;
        waitfor(rate);
        continue;
    end
    if pose_lost
        fprintf('MOCAP pose is back, resuming\n');
        pose_lost = false;
    end

    if abs(xc1) > 50 || abs(yc1) > 50
        fprintf('\nPose (%.1f, %.1f) looks like millimetres; the controller expects metres. Stopping.\n', xc1, yc1);
        cmdMsg.linear.x = 0;
        cmdMsg.angular.z = 0;
        send(cmdPub, cmdMsg);
        return;
    end

    % --------------------------------------------------------
    % Store trajectory
    % --------------------------------------------------------

    trajectory_x(end+1) = xc1;
    trajectory_y(end+1) = yc1;
    trajectory_t(end+1) = toc(t_start);

    % First received position = start position
    if isempty(start_x)
        start_x = xc1;
        start_y = yc1;

        set(startPlot, ...
            'XData', start_x, ...
            'YData', start_y);
    end

    % --------------------------------------------------------
    % Calculate desired heading
    % --------------------------------------------------------

    theta_d = atan2(yd-yc1, xd-xc1);

    % --------------------------------------------------------
    % DT
    % --------------------------------------------------------

    dt = toc(timer);
    timer = tic;

    % Prevent division by zero
    if dt <= 0
        dt = 0.001;
    end
    dt = min(dt, 0.5);      % a long gap must not blow up the I and D terms

    % --------------------------------------------------------
    % Position error
    % --------------------------------------------------------

    ep_c = sqrt((xc1-xd)^2 + (yc1-yd)^2);

    % --------------------------------------------------------
    % Heading error
    % --------------------------------------------------------

    ea_c = theta_d-theta_c;

    % Wrap angle to [-pi, pi]
    ea_c = atan2(sin(ea_c), cos(ea_c));

    % --------------------------------------------------------
    % Stop condition
    % --------------------------------------------------------

    if ep_c < tol

        fprintf('\nGOAL REACHED! (error %.3f m)\n', ep_c);

        % Stop robot
        cmdMsg.linear.x = 0;
        cmdMsg.angular.z = 0;
        send(cmdPub, cmdMsg);

        break;
    end

    % --------------------------------------------------------
    % Derivative and integral terms
    % --------------------------------------------------------

    if first_loop
        ep_prev = ep_c;
        ea_prev = ea_c;
        first_loop = false;
    end

    dep = (ep_c - ep_prev) / dt;                                   % d(ep)/dt
    dea = atan2(sin(ea_c - ea_prev), cos(ea_c - ea_prev)) / dt;    % d(ea)/dt, wrapped
    ei_p_new = min(ei_p + ep_c*dt, 2.0);                           % integral, capped
    ei_a_new = max(-5.0, min(ei_a + ea_c*dt, 5.0));                % integral, capped

    % --------------------------------------------------------
    % Linear velocity PID
    % --------------------------------------------------------

    v_pid = kp_p*ep_c + ki_p*ei_p_new + kd_p*dep;

    % saturate to [0, 0.2], slow down if not facing the goal
    v = max(0, min(v_pid, V_MAX)) * max(0, cos(ea_c));

    % ramp v so the robot does not jerk
    dv = max_accel*dt;
    v = max(v_prev - dv, min(v, v_prev + dv));

    % --------------------------------------------------------
    % Angular velocity PID
    % --------------------------------------------------------

    w_pid = kp_a*ea_c + ki_a*ei_a_new + kd_a*dea;

    % saturate to [-0.2, 0.2]
    omega = max(-W_MAX, min(w_pid, W_MAX));

    % --------------------------------------------------------
    % Update PID states (anti-windup: integrate only when not saturated)
    % --------------------------------------------------------

    ep_prev = ep_c;
    if v_pid < V_MAX
        ei_p = ei_p_new;
    end

    ea_prev = ea_c;
    if abs(w_pid) < W_MAX
        ei_a = ei_a_new;
    end

    v_prev = v;

    % --------------------------------------------------------
    % Send velocity command
    % --------------------------------------------------------

    cmdMsg.linear.x = double(v);
    cmdMsg.angular.z = double(omega);

    send(cmdPub, cmdMsg);

    % ========================================================
    % UPDATE LIVE PLOT
    % ========================================================

    set(trajectoryPlot, ...
        'XData', trajectory_x, ...
        'YData', trajectory_y);

    set(robotPlot, ...
        'XData', xc1, ...
        'YData', yc1);

    % Keep goal visible
    xlim auto;
    ylim auto;

    drawnow limitrate;

    % --------------------------------------------------------
    % Console information
    % --------------------------------------------------------

    fprintf('Position: X = %.3f, Y = %.3f | Error = %.3f | v = %.3f, w = %.3f\n', ...
        xc1, yc1, ep_c, v, omega);

    waitfor(rate);

end


% ============================================================
% FINAL TRAJECTORY PLOT
% ============================================================

figure(2);
clf;

hold on;
grid on;
axis equal;

plot(trajectory_x, trajectory_y, 'b-', ...
    'LineWidth', 2);

plot(start_x, start_y, 'go', ...
    'MarkerSize', 10, ...
    'LineWidth', 2);

plot(xd, yd, 'rx', ...
    'MarkerSize', 12, ...
    'LineWidth', 3);

plot(trajectory_x(end), trajectory_y(end), 'cyo', ...
    'MarkerSize', 8, ...
    'LineWidth', 2);

xlabel('X Position');
ylabel('Y Position');

title('Final Robot Trajectory');

legend('Trajectory', ...
       'Start', ...
       'Goal', ...
       'Final Position');

hold off;


% ============================================================
% SAVE DATA
% ============================================================

trajectory = table( ...
    trajectory_t(:), ...
    trajectory_x(:), ...
    trajectory_y(:), ...
    'VariableNames', {'Time', 'X', 'Y'});

writetable(trajectory, 'robot_trajectory.csv');

fprintf('\nTrajectory saved to robot_trajectory.csv\n');
