clc; clear; close all;
load('data.mat');

% position comparison in 3D
figure(1)
plot3(pos_gt(:,2), pos_gt(:,3), pos_gt(:,4), pos_obs(:,2), pos_obs(:,3), pos_obs(:,4), 'LineWidth', 2)
legend('position of ground truth', 'position of observation')
xlabel('x'); ylabel('y'); zlabel('z');

% velocity comparison in 3D
figure(2)
plot3(vel_gt(:,2), vel_gt(:,3), vel_gt(:,4), vel_obs(:,2), vel_obs(:,3), vel_obs(:,4), 'LineWidth', 2)
legend('velocity of ground truth', 'velocity of observation')
xlabel('vx'); ylabel('vy'); zlabel('vz');

% pos-t
figure(3)
subplot(3,1,1)
plot(pos_gt(:,1), pos_gt(:,2), pos_obs(:,1), pos_obs(:,2), 'LineWidth', 2)
legend('pos_{gt}.x', 'pos_{obs}.x')
xlabel('t'); ylabel('x');
subplot(3,1,2)
plot(pos_gt(:,1), pos_gt(:,3), pos_obs(:,1), pos_obs(:,3), 'LineWidth', 2)
legend('pos_{gt}.y', 'pos_{obs}.y')
xlabel('t'); ylabel('y');
subplot(3,1,3)
plot(pos_gt(:,1), pos_gt(:,4), pos_obs(:,1), pos_obs(:,4), 'LineWidth', 2)
legend('pos_{gt}.z', 'pos_{obs}.z')
xlabel('t'); ylabel('z');

% vel-t
figure(4)
subplot(3,1,1)
plot(vel_gt(:,1), vel_gt(:,2), vel_obs(:,1), vel_obs(:,2), 'LineWidth', 2)
legend('vel_{gt}.x', 'vel_{obs}.x')
xlabel('t'); ylabel('v_x');
subplot(3,1,2)
plot(vel_gt(:,1), vel_gt(:,3), vel_obs(:,1), vel_obs(:,3), 'LineWidth', 2)
legend('vel_{gt}.y', 'vel_{obs}.y')
xlabel('t'); ylabel('v_y');
subplot(3,1,3)
plot(vel_gt(:,1), vel_gt(:,4), vel_obs(:,1), vel_obs(:,4), 'LineWidth', 2)
legend('vel_{gt}.z', 'vel_{obs}.z')
xlabel('t'); ylabel('v_z');