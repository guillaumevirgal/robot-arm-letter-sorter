%---------------------------------------
% Inverse Kinematics for Spatial 4R robot
% Base + Shoulder + Elbow + Wrist + Gripper
% Adapted from B5XRO base code (Kong, 2023)
%---------------------------------------

% Link parameters
d_1 = 20;   % Base height (mm)
l_1 = 25;   % Shoulder to elbow
l_2 = 25;   % Elbow to wrist
l_3 = 5;    % Wrist to gripper tip (gripper length)

%---------------------------------------
% Target pose : position + gripper orientation
%---------------------------------------
x_target = 20;
y_target = 20;
z_target = 10;

% Gripper approach vector (vertical = pointing down to grab a flat letter)
% If gripper points straight down : ux=0, uy=0, uz=-1
ux = x_target / sqrt(x_target^2 + y_target^2);
uy = y_target / sqrt(x_target^2 + y_target^2);
uz = 0;

%---------------------------------------
% Step 1 : Compute wrist center (subtract gripper offset from target)
%---------------------------------------
x = x_target - l_3 * ux;
y = y_target - l_3 * uy;
z = z_target - l_3 * uz;

%---------------------------------------
% Step 2 : Solve theta_1 (base rotation)
%---------------------------------------
xp = sqrt(x^2 + y^2);
zp = z - d_1;

ctheta_1 = x / xp;
stheta_1 = y / xp;
theta_1 = atan2(stheta_1, ctheta_1);

%---------------------------------------
% Step 3 : Solve theta_3 (elbow)
%---------------------------------------
ctheta_3 = (xp^2 + zp^2 - l_1^2 - l_2^2) / (2 * l_1 * l_2);

% Check reachability
if ctheta_3 > 1
    error('Target is out of reach for the robot.');
end
if ctheta_3 < -1
    error('Target is too close for the robot.');
end

% Elbow up configuration
stheta_3 = -sqrt(1 - ctheta_3^2);
% For elbow down : stheta_3 = sqrt(1 - ctheta_3^2);
theta_3 = atan2(stheta_3, ctheta_3);

%---------------------------------------
% Step 4 : Solve theta_2 (shoulder)
%---------------------------------------
sbeta = zp / sqrt(xp^2 + zp^2);
cbeta = xp / sqrt(xp^2 + zp^2);
beta  = atan2(sbeta, cbeta);

cpsi = (xp^2 + zp^2 + l_1^2 - l_2^2) / (2 * l_1 * sqrt(xp^2 + zp^2));
spsi = (l_2 * stheta_3) / sqrt(xp^2 + zp^2);
psi  = atan2(spsi, cpsi);

theta_2 = beta - psi;

%---------------------------------------
% Step 5 : Solve theta_4 (wrist)
% Compensates shoulder + elbow to keep gripper orientation constant
%---------------------------------------
theta_4 = -(theta_2 + theta_3) + pi/2;

%---------------------------------------
% Display results
%---------------------------------------
fprintf('--- Joint Angles ---\n');
fprintf('theta_1 (base)     : %.2f deg\n', rad2deg(theta_1));
fprintf('theta_2 (shoulder) : %.2f deg\n', rad2deg(theta_2));
fprintf('theta_3 (elbow)    : %.2f deg\n', rad2deg(theta_3));
fprintf('theta_4 (wrist)    : %.2f deg\n', rad2deg(theta_4));

%---------------------------------------
% Forward kinematics : compute joint positions for plot
%---------------------------------------
O_1 = [0, 0, 0];
O_2 = [0, 0, d_1];

O_3 = O_2 + l_1 * [cos(theta_2)*cos(theta_1), ...
                    cos(theta_2)*sin(theta_1), ...
                    sin(theta_2)];

O_4 = O_3 + l_2 * [cos(theta_2 + theta_3)*cos(theta_1), ...
                    cos(theta_2 + theta_3)*sin(theta_1), ...
                    sin(theta_2 + theta_3)];

% Gripper tip (end effector)
P = O_4 + l_3 * [ux, uy, uz];

%---------------------------------------
% Plot
%---------------------------------------
points  = [O_1; O_2; O_3; O_4; P];
XX = points(:,1)';
YY = points(:,2)';
ZZ = points(:,3)';

figure;
plot3(XX, YY, ZZ, 'g-o', 'LineWidth', 2, 'MarkerSize', 10);
hold on;

% Highlight target
plot3(x_target, y_target, z_target, 'r*', 'MarkerSize', 15);

% Labels
labels = {'Base', 'Shoulder', 'Elbow', 'Wrist', 'Gripper'};
for i = 1:5
    text(XX(i), YY(i), ZZ(i), ['  ' labels{i}]);
end

xlabel('X'); ylabel('Y'); zlabel('Z');
title('4R Robot Arm - Inverse Kinematics');
grid on; axis equal;
hold off;