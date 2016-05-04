function mysimSwarm_start
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
dt = 2;

% initial channel simulation for comms
channel = initChannel();

% initialise all the agents
numAgents = 3;
agents(numAgents,1)=UAVSimSwarm;
controllers(numAgents,1) = UAVControllerSwarm;

% initial targets to get coverage of the map
intialTargets = [500 -500 500;-500 -500 500];%[500, 500, -500, -500; 500, -500, 500, -500];

% open new figure window
figure
hold on % so each plot doesn't wipe the predecessor

% main simulation loop
for kk=1:1000,
    
    % time
    t = t + dt;
    
%     % simulate received message
%     [rxMsgs{kk},channel] = simReceive(kk,channel);
     
    for j=1:numAgents
        
        % Take sensor readings from UAVSim
        gps = agents(j).gps_sensor();
        p = agents(j).cloud_sensor(cloud, t);
        power = agents(j).update_battery(dt);
        
        % Pass these sensor readings to the controller
        controllers(j).set_current_pos(gps);
        if controllers(j).launched == 0
            controllers(j).target = intialTargets(:,j);
            controllers(j).launched = 1;
        end
%         txtMsg = controller.nav_decision(p,kk, dt);
        controllers(j).nav_decision(p,kk,dt);
        
    end
    
    % Pass the decided velocity and turn curvature back to UAVSim to
    % move     
    for j=1:numAgents
        agents(j).move(controllers(j).u, dt)   
    end
        
    % clear the axes for fresh plotting
    cla
    
    % plot robot locations
    for j=1:numAgents
        plotUAV(agents(j));
    end
    
    % plot the cloud contours
    cloudplot(cloud,t)
    
    % put information in the title
    title(sprintf('t=%.1f secs', t))
    
    % pause ensures that the plots update
    pause(0.01)
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function channel = initChannel()
% initialize comms channel model
channel.curMsgs = {};
channel.newMsgs = {};

function [rxMsgs,channel] = simReceive(aa,channel)
% simulate receiving messages
% simple broadcast model - just get everything
rxMsgs = channel.curMsgs;

function channel = simTransmit(txMsgs,aa,channel)
% simulate transmitting a message
% store it for next step
channel.newMsgs = [channel.newMsgs txMsgs];

function channel = simChannel(channel,x)
% simple - everyone gets everything
channel.curMsgs = channel.newMsgs;
channel.newMsgs = {};