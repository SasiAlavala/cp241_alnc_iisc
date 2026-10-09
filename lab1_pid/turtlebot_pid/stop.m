% CURRENT STATE
setenv("ROS_DOMAIN_ID","30");

disp("Creating ROS 2 node...");
node = ros2node("/matlab_controller");
disp("Node created.");

% CURRENT STATE
disp("Checking topics...");
topics = ros2("topic","list");
disp(topics);

% CURRENT STATE
disp("Creating publisher...");
pub = ros2publisher( ...
    node, ...
    "/HWTB3_10/cmd_vel", ...
    "geometry_msgs/Twist");
disp("Publisher created.");

% CURRENT STATE
disp("Creating Twist message...");
msg = ros2message(pub);

% CURRENT STATE
msg.linear.x = 0.0;
msg.linear.y = 0.0;
msg.linear.z = 0.0;

msg.angular.x = 0.0;
msg.angular.y = 0.0;
msg.angular.z = 0.0;

disp("Starting continuous publishing...");
disp("Press Ctrl+C to stop.");

% CURRENT STATE
count = 0;

while true

    send(pub,msg);

    count = count + 1;

    if mod(count,10) == 0
        fprintf("Publishing cmd_vel | Message #%d | angular.z = %.2f\n", ...
            count, msg.angular.z);
    end

    pause(0.1);
end
