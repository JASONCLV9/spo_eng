clear; close all; clc;

data = readmatrix("left_hand_height.csv");

time = data(:, 1);
positions = data(:, 3);

figure;
plot(time, positions, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Left hand height [m]');
grid on;
xlim([0, max(time)])


data = readmatrix("elbow_flex_l.csv");

time = data(:, 2);
positions = data(:, 3);

figure;
plot(time, positions, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Left elbow flexion [deg]');
grid on;
xlim([0, max(time)])