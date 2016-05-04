function mysim_start
%
% simulation example for use of cloud dispersion model
%
% Arthur Richards, Nov 2014
%

% load cloud data
% choose a scenario
% load 'cloud1.mat'
load 'cloud2.mat'

% time and time step
t = 0;
dt = 4;

uav = UAVSim();
controller = UAVController();

% open new figure window
figure
hold on % so each plot doesn't wipte the predecessor

% main simulation loop
for kk=1:1000,
    
    % time
    t = t + dt;
    
    %sensors
    gps = uav.gps_sensor();
    p = uav.cloud_sensor(cloud, t);
    power = uav.update_battery(dt);
    
    controller.set_current_pos(gps);    
    
    u = controller.nav_decision(p,kk, dt);

    uav.move(u, dt)
    % clear the axes for fresh plotting
    cla
    
    % put information in the title
    title(sprintf('t=%.1f secs Concentration=%.2f', t, p))
        
    % plot robot location
    plotUAV(uav)
    
    % plot the cloud contours
    cloudplot(cloud,t)
    
    % pause ensures that the plots update
    pause(0.1)
    
end
