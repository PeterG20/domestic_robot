clear
Sample_T=0.001; % Simulation Sampling Time
timescale=0:Sample_T:20;
timescale=timescale(2:end);
[q sizeT]=size(timescale);

%Path Parameters
V=[1*ones(sizeT,1)];
R=[2.*ones(sizeT,1)];


V = timeseries(V, timescale);
close all
R = timeseries(R, timescale);


r=0.05; % Track radious
l=0.6 % Distance betwween tracks
B=0.4 % Raw Caterpillar Rate
R=R*3;

Ka=1; %% Dynamics Gain
Tau=0.4; %% Dynamics Tau

Slip_power=0.3;
Slip_time=0.25;
%% NO Slip
Slip_on=1;%% Slip Disabled
Dynamics_on=0;%% Dynamics Enabled
out=sim("Robot_Model_2.slx",max(timescale))
% plot

figure
plot(out.xg_var,out.yg_var,'linewidth',3.5, 'Color', 'r') % really red line
title('Trajectory of Robot without slipage','FontSize',30)
xlabel('Global Position X [m]','FontSize',30); 
ylabel('Global Position Y [m]','FontSize',30);
grid on;

% change the color of the x and y axis lines to black and line width to 2
set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)

% change the font size of the x and y tick labels
set(gca, 'FontSize', 25)

% create the legend and set its font size
lgd = legend('Trajectory of Robot');
lgd.FontSize = 25;

% make the grid more transparent
set(gca, 'GridAlpha', 0.02, 'MinorGridAlpha', 0.02);

% set y-axis limits
ylim([0 3]);

% set the number of steps/values on the y-axis
set(gca, 'YTick', 0:1:3);

% set y-axis limits
xlim([-3 3]);

% set the number of steps/values on the y-axis
set(gca, 'XTick', -3:1:3);
axis equal;set(gca,'DataAspectRatioMode','manual');set(gca,'DataAspectRatio',[1 1 1]);set(gca,'PlotBoxAspectRatioMode','manual');set(gca,'PlotBoxAspectRatio',[1 1 1]);

figure
% Plot of global positions (xg and yg)
subplot(2, 2, 1)
plot(out.xg_var, out.yg_var, 'r', 'linewidth', 3.5)
title('Global Position', 'FontSize', 25)
xlabel('Global Position X [m]', 'FontSize', 20)
ylabel('Global Position Y [m]', 'FontSize', 20)
grid on
ylim([min(out.yg_var)*1.25 max(out.yg_var)*1.25]);
xlim([min(out.xg_var)*1.25 max(out.xg_var)*1.25]);
% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of angular velocities (wi and wo)
axis equal;set(gca,'DataAspectRatioMode','manual');set(gca,'DataAspectRatio',[1 1 1]);set(gca,'PlotBoxAspectRatioMode','manual');set(gca,'PlotBoxAspectRatio',[1 1 1]);

subplot(2, 2, 2)
plot(out.tout, out.wi_var, 'b', out.tout, out.wo_var, 'g', 'linewidth', 2)
title('Angular Velocities', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Angular Velocity [rad/s]', 'FontSize', 20)
legend('wi', 'wo', 'FontSize', 20)
grid on
ylim([17 23]);
xlim([0 20]);

% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of speeds for inner and outer tracks (Vi and Vo)
subplot(2, 2, 3)
plot(out.tout, out.Vi_var, 'b', out.tout, out.Vo_var, 'g', 'linewidth', 2)
title('Track Speeds', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Speed [m/s]', 'FontSize', 20)
legend('Vi', 'Vo', 'FontSize', 20)
grid on
% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of desired turning radius (Rout)
subplot(2, 2, 4)
plot(R./3, 'b', 'linewidth', 2)
title('Turning Radius', 'FontSize', 25)
hold on;
plot(out.tout, out.Rout__var./3, 'm', 'linewidth', 2)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Radius [m]', 'FontSize', 20)
legend('Desired Turning Radius', 'Actual Turning Radius', 'FontSize', 20)
grid on
ylim([min(out.Rout__var./3)*0.25 max(out.Rout__var./3)*1.25]);

% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)

figure
% Plot of local positions (xl and yl)
subplot(2, 2, 2)
plot(out.xl_var, out.yl_var, 'r', 'linewidth', 3.5)
title('Local Position', 'FontSize', 25)
xlabel('Local Position X [m]', 'FontSize', 20)
ylabel('Local Position Y [m]', 'FontSize', 20)
grid on
% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of global velocity (Vvg)
axis equal;set(gca,'DataAspectRatioMode','manual');set(gca,'DataAspectRatio',[1 1 1]);set(gca,'PlotBoxAspectRatioMode','manual');set(gca,'PlotBoxAspectRatio',[1 1 1]);

subplot(2, 2, 3)
plot(out.tout, out.Vvg_var, 'b', 'linewidth', 2)
title('Global Velocity', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Velocity [m/s]', 'FontSize', 20)
grid on
% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of angular displacements (phig and phil)
subplot(2, 2, 4)
plot(out.tout, out.phig_var, 'g', out.tout, out.phil_var, 'm', 'linewidth', 2)
title('Angular Displacements', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Angular Displacement [rad]', 'FontSize', 20)
legend('phig', 'phil', 'FontSize', 20)
grid on
% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
subplot(2, 2, 1)
plot(out.xg_var, out.yg_var, 'r', 'linewidth', 2)
title('Global Position', 'FontSize', 25)
xlabel('Global Position X [m]', 'FontSize', 20)
ylabel('Global Position Y [m]', 'FontSize', 20)
grid on

% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
axis equal;set(gca,'DataAspectRatioMode','manual');set(gca,'DataAspectRatio',[1 1 1]);set(gca,'PlotBoxAspectRatioMode','manual');set(gca,'PlotBoxAspectRatio',[1 1 1]);

%%  Slip
Slip_on=0;%% Slip Enabled

out=sim("Robot_Model_2.slx",max(timescale))
% plot

figure
plot(out.xg_var,out.yg_var,'linewidth',1.5, 'Color', 'r') % really red line
title('Trajectory of Robot with Slipage','FontSize',30)
xlabel('Global Position X [m]','FontSize',30); 
ylabel('Global Position Y [m]','FontSize',30);
grid on;

% change the color of the x and y axis lines to black and line width to 2
set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)

% change the font size of the x and y tick labels
set(gca, 'FontSize', 25)

% create the legend and set its font size
lgd = legend('Trajectory of Robot');
lgd.FontSize = 25;

% make the grid more transparent
set(gca, 'GridAlpha', 0.02, 'MinorGridAlpha', 0.02);

% set y-axis limits
ylim([0 3]);

% set the number of steps/values on the y-axis
set(gca, 'YTick', 0:1:3);

% set y-axis limits
xlim([-3 3]);

% set the number of steps/values on the y-axis
set(gca, 'XTick', -3:1:3);
axis equal
 set(gca,'DataAspectRatioMode','manual')
 set(gca,'DataAspectRatio',[1 1 1])
 set(gca,'PlotBoxAspectRatioMode','manual')
 set(gca,'PlotBoxAspectRatio',[1 1 1])


% New figure
figure
plot(out.xg_var,out.yg_var,'linewidth',3.5, 'Color', 'r') % really red line
title('Trajectory of Robot without slipage','FontSize',30)
xlabel('Global Position X [m]','FontSize',30); 
ylabel('Global Position Y [m]','FontSize',30);
grid on;

% change the color of the x and y axis lines to black and line width to 2
set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)

% change the font size of the x and y tick labels
set(gca, 'FontSize', 25)

% create the legend and set its font size
lgd = legend('Trajectory of Robot');
lgd.FontSize = 25;

% make the grid more transparent
set(gca, 'GridAlpha', 0.02, 'MinorGridAlpha', 0.02);

% set y-axis limits
ylim([0 3]);

% set the number of steps/values on the y-axis
set(gca, 'YTick', 0:1:3);

% set y-axis limits
xlim([-3 3]);

% set the number of steps/values on the y-axis
set(gca, 'XTick', -3:1:3);
axis equal
 set(gca,'DataAspectRatioMode','manual')
 set(gca,'DataAspectRatio',[1 1 1])
 set(gca,'PlotBoxAspectRatioMode','manual')
 set(gca,'PlotBoxAspectRatio',[1 1 1])

figure
% Plot of global positions (xg and yg)
subplot(2, 2, 1)
plot(out.xg_var, out.yg_var, 'r', 'linewidth', 3.5)
title('Global Position', 'FontSize', 25)
xlabel('Global Position X [m]', 'FontSize', 20)
ylabel('Global Position Y [m]', 'FontSize', 20)
grid on
ylim([min(out.yg_var)*1.25 max(out.yg_var)*1.25]);
xlim([min(out.xg_var)*1.25 max(out.xg_var)*1.25]);
% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of angular velocities (wi and wo)
axis equal;set(gca,'DataAspectRatioMode','manual');set(gca,'DataAspectRatio',[1 1 1]);set(gca,'PlotBoxAspectRatioMode','manual');set(gca,'PlotBoxAspectRatio',[1 1 1]);

subplot(2, 2, 2)
plot(out.tout, out.wi_var, 'b', out.tout, out.wo_var, 'g', 'linewidth', 2)
title('Angular Velocities', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Angular Velocity [rad/s]', 'FontSize', 20)
legend('wi', 'wo', 'FontSize', 20)
grid on
ylim([17 23]);
xlim([0 20]);

% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of speeds for inner and outer tracks (Vi and Vo)
subplot(2, 2, 3)
plot(out.tout, out.Vi_var, 'b', out.tout, out.Vo_var, 'g', 'linewidth', 2)
title('Track Speeds', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Speed [m/s]', 'FontSize', 20)
legend('Vi', 'Vo', 'FontSize', 20)
grid on
% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of desired turning radius (Rout)
subplot(2, 2, 4)
plot(R./3, 'b', 'linewidth', 2)
title('Turning Radius', 'FontSize', 25)
hold on;
plot(out.tout, out.Rout__var./3, 'm', 'linewidth', 2)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Radius [m]', 'FontSize', 20)
legend('Desired Turning Radius', 'Actual Turning Radius', 'FontSize', 20)

grid on
ylim([min(out.Rout__var./3)*1.25 max(out.Rout__var./3)*1.25]);

% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)

figure
% Plot of local positions (xl and yl)
subplot(2, 2, 2)
plot(out.xl_var, out.yl_var, 'r', 'linewidth', 3.5)
title('Local Position', 'FontSize', 25)
xlabel('Local Position X [m]', 'FontSize', 20)
ylabel('Local Position Y [m]', 'FontSize', 20)
grid on
% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
axis equal;set(gca,'DataAspectRatioMode','manual');set(gca,'DataAspectRatio',[1 1 1]);set(gca,'PlotBoxAspectRatioMode','manual');set(gca,'PlotBoxAspectRatio',[1 1 1]);

% Plot of global velocity (Vvg)
subplot(2, 2, 3)
plot(out.tout, out.Vvg_var, 'b', 'linewidth', 2)
title('Global Velocity', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Velocity [m/s]', 'FontSize', 20)
grid on
% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of angular displacements (phig and phil)
subplot(2, 2, 4)
plot(out.tout, out.phig_var, 'g', out.tout, out.phil_var, 'm', 'linewidth', 2)
title('Angular Displacements', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Angular Displacement [rad]', 'FontSize', 20)
legend('phig', 'phil', 'FontSize', 20)
grid on
% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
subplot(2, 2, 1)
plot(out.xg_var, out.yg_var, 'r', 'linewidth', 2)
title('Global Position', 'FontSize', 25)
xlabel('Global Position X [m]', 'FontSize', 20)
ylabel('Global Position Y [m]', 'FontSize', 20)
grid on

% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
axis equal;set(gca,'DataAspectRatioMode','manual');set(gca,'DataAspectRatio',[1 1 1]);set(gca,'PlotBoxAspectRatioMode','manual');set(gca,'PlotBoxAspectRatio',[1 1 1]);

%% 2nd Path
Slip_on=1;%% Slip Disabled
Sample_T=0.001; % Simulation Sampling Time
timescale=0:Sample_T:20;
timescale=timescale(2:end)
[q sizeT]=size(timescale);

%Path Parameters
V=[1*ones(sizeT,1)];

%R=[1.*ones(sizeT*0.18,1);-3.*ones(sizeT*0.12,1);10000000.*ones(sizeT*0.10,1);1.*ones(sizeT*0.16,1);10000000.*ones(sizeT*0.2,1);-2.*ones(sizeT*0.22,1)];

Rpath = [1 0.157; 
        100000 0.243;
         1 0.157;
        100000 0.243;
        -1 0.1;
        -3 0.1]; % Radius values and their durations

R = []; % Initialize the R vector

% For each row in Rpath
for i = 1:size(Rpath, 1)
    % Generate a vector with the radius value repeated for its duration
    R_segment = Rpath(i, 1) * ones(round(sizeT * Rpath(i, 2)), 1);
    
    % Append the segment to the R vector
    R = [R; R_segment];
end

V = timeseries(V, timescale);
R = timeseries(R, timescale);


r=0.05; % Track radious
l=0.6 % Distance betwween tracks
B=0.4 % Raw Caterpillar Rate
R=R*3;
% NO Slip
out=sim("Robot_Model_2.slx",max(timescale))
% plot

figure
plot(out.xg_var,out.yg_var,'linewidth',3.5, 'Color', 'r') % really red line
title('Trajectory of Robot without slipage','FontSize',30)
xlabel('Global Position X [m]','FontSize',30); 
ylabel('Global Position Y [m]','FontSize',30);
grid on;

% change the color of the x and y axis lines to black and line width to 2
set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)

% change the font size of the x and y tick labels
set(gca, 'FontSize', 25)

% create the legend and set its font size
lgd = legend('Trajectory of Robot');
lgd.FontSize = 25;

% make the grid more transparent
set(gca, 'GridAlpha', 0.02, 'MinorGridAlpha', 0.02);

% set y-axis limits
%ylim([-3 3]);

% set the number of steps/values on the y-axis
%set(gca, 'YTick', -3:1:3);

% set y-axis limits
%xlim([-3 3]);

% set the number of steps/values on the y-axis
%set(gca, 'XTick', -3:1:3);
axis equal
 set(gca,'DataAspectRatioMode','manual')
 set(gca,'DataAspectRatio',[1 1 1])
 set(gca,'PlotBoxAspectRatioMode','manual')
 set(gca,'PlotBoxAspectRatio',[1 1 1])
% New figure
figure
plot(out.xg_var,out.yg_var,'linewidth',3.5, 'Color', 'r') % really red line
title('Trajectory of Robot without slipage','FontSize',30)
xlabel('Global Position X [m]','FontSize',30); 
ylabel('Global Position Y [m]','FontSize',30);
grid on;

% change the color of the x and y axis lines to black and line width to 2
set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)

% change the font size of the x and y tick labels
set(gca, 'FontSize', 25)

% create the legend and set its font size
lgd = legend('Trajectory of Robot');
lgd.FontSize = 25;

% make the grid more transparent
set(gca, 'GridAlpha', 0.02, 'MinorGridAlpha', 0.02);

% set y-axis limits
ylim([0 3]);

% set the number of steps/values on the y-axis
set(gca, 'YTick', 0:1:3);

% set y-axis limits
xlim([-3 3]);

% set the number of steps/values on the y-axis
set(gca, 'XTick', -3:1:3);
axis equal
 set(gca,'DataAspectRatioMode','manual')
 set(gca,'DataAspectRatio',[1 1 1])
 set(gca,'PlotBoxAspectRatioMode','manual')
 set(gca,'PlotBoxAspectRatio',[1 1 1])

figure
% Plot of global positions (xg and yg)
subplot(2, 2, 1)
plot(out.xg_var, out.yg_var, 'r', 'linewidth', 3.5)
title('Global Position', 'FontSize', 25)
xlabel('Global Position X [m]', 'FontSize', 20)
ylabel('Global Position Y [m]', 'FontSize', 20)
grid on
ylim([min(out.yg_var)*1.25 max(out.yg_var)*1.25]);
xlim([min(out.xg_var)*1.25 max(out.xg_var)*1.25]);
% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
axis equal;set(gca,'DataAspectRatioMode','manual');set(gca,'DataAspectRatio',[1 1 1]);set(gca,'PlotBoxAspectRatioMode','manual');set(gca,'PlotBoxAspectRatio',[1 1 1]);

% Plot of angular velocities (wi and wo)
subplot(2, 2, 2)
plot(out.tout, out.wi_var, 'b', out.tout, out.wo_var, 'g', 'linewidth', 2)
title('Angular Velocities', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Angular Velocity [rad/s]', 'FontSize', 20)
legend('wi', 'wo', 'FontSize', 20)
grid on
ylim([17 23]);
xlim([0 20]);

% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of speeds for inner and outer tracks (Vi and Vo)
subplot(2, 2, 3)
plot(out.tout, out.Vi_var, 'b', out.tout, out.Vo_var, 'g', 'linewidth', 2)
title('Track Speeds', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Speed [m/s]', 'FontSize', 20)
legend('Vi', 'Vo', 'FontSize', 20)
grid on
% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of desired turning radius (Rout)
subplot(2, 2, 4)
plot(R./3, 'b', 'linewidth', 2)
title('Turning Radius', 'FontSize', 25)
hold on;
plot(out.tout, out.Rout__var./3, 'm', 'linewidth', 2)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Radius [m]', 'FontSize', 20)
legend('Desired Turning Radius', 'Actual Turning Radius', 'FontSize', 20)

grid on
ylim([min(out.Rout__var./3)*1.25 max(out.Rout__var./3)*1.25]);

% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)

figure
% Plot of local positions (xl and yl)
subplot(2, 2, 2)
plot(out.xl_var, out.yl_var, 'r', 'linewidth', 3.5)
title('Local Position', 'FontSize', 25)
xlabel('Local Position X [m]', 'FontSize', 20)
ylabel('Local Position Y [m]', 'FontSize', 20)
grid on
% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
axis equal;set(gca,'DataAspectRatioMode','manual');set(gca,'DataAspectRatio',[1 1 1]);set(gca,'PlotBoxAspectRatioMode','manual');set(gca,'PlotBoxAspectRatio',[1 1 1]);

% Plot of global velocity (Vvg)
subplot(2, 2, 3)
plot(out.tout, out.Vvg_var, 'b', 'linewidth', 2)
title('Global Velocity', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Velocity [m/s]', 'FontSize', 20)
grid on
% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of angular displacements (phig and phil)
subplot(2, 2, 4)
plot(out.tout, out.phig_var, 'g', out.tout, out.phil_var, 'm', 'linewidth', 2)
title('Angular Displacements', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Angular Displacement [rad]', 'FontSize', 20)
legend('phig', 'phil', 'FontSize', 20)
grid on
% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
subplot(2, 2, 1)
plot(out.xg_var, out.yg_var, 'r', 'linewidth', 2)
title('Global Position', 'FontSize', 25)
xlabel('Global Position X [m]', 'FontSize', 20)
ylabel('Global Position Y [m]', 'FontSize', 20)
grid on

% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
axis equal;set(gca,'DataAspectRatioMode','manual');set(gca,'DataAspectRatio',[1 1 1]);set(gca,'PlotBoxAspectRatioMode','manual');set(gca,'PlotBoxAspectRatio',[1 1 1]);


%% Slip
Slip_on=0;%% Slip Enabled

out=sim("Robot_Model_2.slx",max(timescale))
% plot

figure
plot(out.xg_var,out.yg_var,'linewidth',1.5, 'Color', 'r') % really red line
title('Trajectory of Robot with Slipage','FontSize',30)
xlabel('Global Position X [m]','FontSize',30); 
ylabel('Global Position Y [m]','FontSize',30);
grid on;

% change the color of the x and y axis lines to black and line width to 2
set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)

% change the font size of the x and y tick labels
set(gca, 'FontSize', 25)

% create the legend and set its font size
lgd = legend('Trajectory of Robot');
lgd.FontSize = 25;

% make the grid more transparent
set(gca, 'GridAlpha', 0.02, 'MinorGridAlpha', 0.02);

% set y-axis limits
%ylim([-3 3]);

% set the number of steps/values on the y-axis
%set(gca, 'YTick', -3:1:3);

% set y-axis limits
%xlim([-3 3]);

% set the number of steps/values on the y-axis
%set(gca, 'XTick', -3:1:3);
axis equal
 set(gca,'DataAspectRatioMode','manual')
 set(gca,'DataAspectRatio',[1 1 1])
 set(gca,'PlotBoxAspectRatioMode','manual')
 set(gca,'PlotBoxAspectRatio',[1 1 1])
% New figure
figure
plot(out.xg_var,out.yg_var,'linewidth',3.5, 'Color', 'r') % really red line
title('Trajectory of Robot without slipage','FontSize',30)
xlabel('Global Position X [m]','FontSize',30); 
ylabel('Global Position Y [m]','FontSize',30);
grid on;

% change the color of the x and y axis lines to black and line width to 2
set(gca, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)

% change the font size of the x and y tick labels
set(gca, 'FontSize', 25)

% create the legend and set its font size
lgd = legend('Trajectory of Robot');
lgd.FontSize = 25;

% make the grid more transparent
set(gca, 'GridAlpha', 0.02, 'MinorGridAlpha', 0.02);

% set y-axis limits
ylim([0 3]);

% set the number of steps/values on the y-axis
set(gca, 'YTick', 0:1:3);

% set y-axis limits
xlim([-3 3]);

% set the number of steps/values on the y-axis
set(gca, 'XTick', -3:1:3);
axis equal
 set(gca,'DataAspectRatioMode','manual')
 set(gca,'DataAspectRatio',[1 1 1])
 set(gca,'PlotBoxAspectRatioMode','manual')
 set(gca,'PlotBoxAspectRatio',[1 1 1])

figure
% Plot of global positions (xg and yg)
subplot(2, 2, 1)
plot(out.xg_var, out.yg_var, 'r', 'linewidth', 3.5)
title('Global Position', 'FontSize', 25)
xlabel('Global Position X [m]', 'FontSize', 20)
ylabel('Global Position Y [m]', 'FontSize', 20)
grid on
ylim([min(out.yg_var)*1.25 max(out.yg_var)*1.25]);
xlim([min(out.xg_var)*1.25 max(out.xg_var)*1.25]);
% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
axis equal;set(gca,'DataAspectRatioMode','manual');set(gca,'DataAspectRatio',[1 1 1]);set(gca,'PlotBoxAspectRatioMode','manual');set(gca,'PlotBoxAspectRatio',[1 1 1]);

% Plot of angular velocities (wi and wo)
subplot(2, 2, 2)
plot(out.tout, out.wi_var, 'b', out.tout, out.wo_var, 'g', 'linewidth', 2)
title('Angular Velocities', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Angular Velocity [rad/s]', 'FontSize', 20)
legend('wi', 'wo', 'FontSize', 20)
grid on
ylim([17 23]);
xlim([0 20]);

% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of speeds for inner and outer tracks (Vi and Vo)
subplot(2, 2, 3)
plot(out.tout, out.Vi_var, 'b', out.tout, out.Vo_var, 'g', 'linewidth', 2)
title('Track Speeds', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Speed [m/s]', 'FontSize', 20)
legend('Vi', 'Vo', 'FontSize', 20)
grid on
% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of desired turning radius (Rout)
subplot(2, 2, 4)
plot(out.tout, out.Rout__var./3, 'm', 'linewidth', 2)
plot(R./3, 'b', 'linewidth', 2)
title('Turning Radius', 'FontSize', 25)
hold on;
plot(out.tout, out.Rout__var./3, 'm', 'linewidth', 2)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Radius [m]', 'FontSize', 20)
legend('Desired Turning Radius', 'Actual Turning Radius', 'FontSize', 20)

grid on
ylim([min(out.Rout__var./3)*1.25 max(out.Rout__var./3)*1.25]);

% Global settings for figure 1
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)

figure
% Plot of local positions (xl and yl)
subplot(2, 2, 2)
plot(out.xl_var, out.yl_var, 'r', 'linewidth', 3.5)
title('Local Position', 'FontSize', 25)
xlabel('Local Position X [m]', 'FontSize', 20)
ylabel('Local Position Y [m]', 'FontSize', 20)
grid on
% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
axis equal;set(gca,'DataAspectRatioMode','manual');set(gca,'DataAspectRatio',[1 1 1]);set(gca,'PlotBoxAspectRatioMode','manual');set(gca,'PlotBoxAspectRatio',[1 1 1]);

% Plot of global velocity (Vvg)
subplot(2, 2, 3)
plot(out.tout, out.Vvg_var, 'b', 'linewidth', 2)
title('Global Velocity', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Velocity [m/s]', 'FontSize', 20)
grid on
% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
% Plot of angular displacements (phig and phil)
subplot(2, 2, 4)
plot(out.tout, out.phig_var, 'g', out.tout, out.phil_var, 'm', 'linewidth', 2)
title('Angular Displacements', 'FontSize', 25)
xlabel('Time [s]', 'FontSize', 20)
ylabel('Angular Displacement [rad]', 'FontSize', 20)
legend('phig', 'phil', 'FontSize', 20)
grid on
% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)
subplot(2, 2, 1)
plot(out.xg_var, out.yg_var, 'r', 'linewidth', 2)
title('Global Position', 'FontSize', 25)
xlabel('Global Position X [m]', 'FontSize', 20)
ylabel('Global Position Y [m]', 'FontSize', 20)
grid on

% Global settings for figure 2
set(gca, 'FontSize', 15, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.2)
set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.15)

axis equal;set(gca,'DataAspectRatioMode','manual');set(gca,'DataAspectRatio',[1 1 1]);set(gca,'PlotBoxAspectRatioMode','manual');set(gca,'PlotBoxAspectRatio',[1 1 1]);
