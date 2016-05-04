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



% initialise all the agents
numAgents = 4;
agents(numAgents,1)=UAVSimSwarm;
controllers(numAgents,1) = UAVControllerSwarm;

% initial channel simulation for comms
channel = initChannel();

% initial targets to get coverage of the map
intialTargets = [500, 500, -500, -500; 500, -500, 500, -500];

% open new figure window
figure
hold on % so each plot doesn't wipe the predecessor

% main simulation loop
for kk=1:1000,
    
    % time
    t = t + dt;
         
    for aa=1:numAgents
        
        % Take sensor readings from UAVSim
        gps = agents(aa).gps_sensor();
        p = agents(aa).cloud_sensor(cloud, t);
        power = agents(aa).update_battery(dt);
        
        % Pass these sensor readings to the controller
        controllers(aa).set_current_pos(gps);
        if controllers(aa).launched == 0
            agents(aa).id = aa;
            controllers(aa).id = aa;
            controllers(aa).target = intialTargets(:,aa);
            controllers(aa).launched = 1;
        end
        
        if agents(aa).id ~= controllers(aa).id
            display('Agents and controllers are mixed up');
            break;
        end
        
        % simulate received message
        [rxMsgs,channel] = simReceive(channel);
        
        txMsgs = controllers(aa).nav_decision(p,kk, dt, rxMsgs);
        
        % simulate transmission
        channel = simTransmit(txMsgs,channel);
        
    end
    
    % simulate the comms
    channel = simChannel(channel);
    
    % Pass the decided velocity and turn curvature back to UAVSim to
    % move     
    for aa=1:numAgents
        agents(aa).move(controllers(aa).u, dt)   
    end
        
    % clear the axes for fresh plotting
    cla
    
    % plot robot locations
    for aa=1:numAgents
        plotUAV(agents(aa));
    end
    
    % plot the cloud contours
    cloudplot(cloud,t)
    
    % put information in the title
    title(sprintf('t=%.1f secs', t))
    
    % pause ensures that the plots update
    pause(0.01)
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Communications from Arthur Richards
function channel = initChannel()
% initialize comms channel model
channel.curMsgs = [];
channel.newMsgs = [];

function [rxMsgs,channel] = simReceive(channel)
% simulate receiving messages
% simple broadcast model - just get everything
rxMsgs = channel.curMsgs;

function channel = simTransmit(txMsgs,channel)
% simulate transmitting a message
% store it for next step
channel.newMsgs = [channel.newMsgs txMsgs];

function channel = simChannel(channel)
% simple - everyone gets everything
channel.curMsgs = channel.newMsgs;
channel.newMsgs = [];