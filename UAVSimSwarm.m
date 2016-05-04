classdef UAVSimSwarm < handle
    %UAVSim Class  
    properties (SetAccess = private,GetAccess = private)
        x; %actual position in x
        y; %actual position in y
        heading;
        history;
    end
    
    properties 
%         controller;
        battery;
        id;
        
    end

    methods
        % Constructor
        function uav = UAVSimSwarm()
            uav.x = 0;
            uav.y = 0;
            uav.heading = 0;
            uav.history = [0,0];
%             uav.controller = UAVControllerSwarm();
            uav.battery = 3600;
        end
       
        % Take a sensor reading with the gps to find x and y positions
        function g = gps_sensor(uav)
            g = zeros(3,1);
            g(1) = uav.x + (rand()-0.5)*6; %plus/minus 3m gps noise
            g(2) = uav.y + (rand()-0.5)*6; %plus/minus 3m gps noise
        end
            
        %Take sensor reading of the pollution levels
        function p = cloud_sensor(uav, cloud, t)
            p = cloudsamp(cloud,uav.x,uav.y,t);
        end     
        
        %Update the remaining power in the batter and return
        function power = update_battery(uav, dt)
            uav.battery = uav.battery - dt;
            power = uav.battery;
        end
            
        function move(uav, u, dt)
            if u(1)<10
                u(1) = 10;
            elseif u(1)>20
                u(1)=20;
            end
            if u(2)<-6
                u(2) = -6;
            elseif u(2)>6
                u(2) = 6;
            end  
            state = [uav.x; uav.y; uav.heading];
            stateNew = runge_kutta4(state, u, dt);
            uav.x = stateNew(1);
            uav.y = stateNew(2);
            uav.heading = stateNew(3);
        end
        
        function plotUAV(uav)
            if uav.battery>0 %Don't plot UAV if battery has died
                plot(uav.x, uav.y, 'o')
%                 plot(uav.history(:,1), uav.history(:,2), '.') %Uncomment
%                 to see path
                uav.history = [uav.history; uav.x, uav.y];
            end
        end
    end
    
end

