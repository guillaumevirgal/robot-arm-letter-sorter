%---------------------------------------
% Inverse Kinematics for 4-DOF robot arm
% Base yaw + Shoulder pitch + L1 roll + Elbow pitch
%---------------------------------------

% Link parameters (mm)
L_0 = 200;   % Base height
L_1 = 200;   % Shoulder to elbow
L_2 = 185;   % Elbow to gripper tip (L2 + gripper, rigid)

%---------------------------------------
% Target position
%---------------------------------------
x_target = 270; %Tray (325,0,150)
y_target = 0;   %Vision (270,0,160)
z_target = 160; %

%---------------------------------------
% Step 1 : theta_1 (base yaw)
%---------------------------------------
theta_1 = atan2(y_target, x_target);

%---------------------------------------
% Step 2 : Project to radial-vertical plane
%---------------------------------------
r = sqrt(x_target^2 + y_target^2);
z = z_target - L_0;
D = sqrt(r^2 + z^2);

%---------------------------------------
% Step 3 : theta_3 (elbow) via law of cosines
%---------------------------------------
cos_theta_3 = (D^2 - L_1^2 - L_2^2) / (2 * L_1 * L_2);
sin_theta_3 = -sqrt(1 - cos_theta_3^2);  % elbow up
theta_3 = atan2(sin_theta_3, cos_theta_3);

%---------------------------------------
% Step 4 : theta_2 (shoulder)
%---------------------------------------
cos_beta = r / D;
sin_beta = z / D;
beta = atan2(sin_beta, cos_beta);

sin_psi = (L_2 * sin_theta_3) / D;
cos_psi = (L_1^2 + D^2 - L_2^2) / (2 * L_1 * D);
psi = atan2(sin_psi, cos_psi);

theta_2 = beta - psi;

%---------------------------------------
% Step 5 : theta_roll (L1 axial roll)
%---------------------------------------
theta_roll = 0;  % radial reach

%---------------------------------------
% Display results
%---------------------------------------
fprintf('--- Joint Angles ---\n');
fprintf('theta_1 (base yaw)      : %.2f deg\n', rad2deg(theta_1));
fprintf('theta_2 (shoulder pitch) : %.2f deg\n', rad2deg(theta_2));
fprintf('theta_roll (L1 roll)     : %.2f deg\n', rad2deg(theta_roll));
fprintf('theta_3 (elbow pitch)    : %.2f deg\n', rad2deg(theta_3));

%---------------------------------------
% Forward kinematics verification
%---------------------------------------
shoulder = [0, 0, L_0];

elbow = shoulder + L_1 * [cos(theta_2)*cos(theta_1), ...
                          cos(theta_2)*sin(theta_1), ...
                          sin(theta_2)];

tip = elbow + L_2 * [cos(theta_2 + theta_3)*cos(theta_1), ...
                     cos(theta_2 + theta_3)*sin(theta_1), ...
                     sin(theta_2 + theta_3)];

fprintf('\n--- FK Verification ---\n');
fprintf('Target : (%.1f, %.1f, %.1f)\n', x_target, y_target, z_target);
fprintf('FK tip : (%.1f, %.1f, %.1f)\n', tip(1), tip(2), tip(3));
fprintf('Error  : %.2f mm\n', norm(tip - [x_target, y_target, z_target]));

%---------------------------------------
% Plot
%---------------------------------------
points = [[0,0,0]; shoulder; elbow; tip];
XX = points(:,1)';
YY = points(:,2)';
ZZ = points(:,3)';

figure;
plot3(XX, YY, ZZ, 'g-o', 'LineWidth', 2, 'MarkerSize', 10);
hold on;
plot3(x_target, y_target, z_target, 'r*', 'MarkerSize', 15);

labels = {'Base', 'Shoulder', 'Elbow', 'Tip'};
for i = 1:4
    text(XX(i), YY(i), ZZ(i), ['  ' labels{i}]);
end

xlabel('X'); ylabel('Y'); zlabel('Z');
title('3-DOF Robot Arm - Inverse Kinematics');
grid on; axis equal;
hold off;
