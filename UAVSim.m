classdef UAVSim < handle
    %UAVSim Class  
    properties (SetAccess = private,GetAccess = private)
        x; %actual position in x
        y; %actual position in y
    end
    
    properties (SetAccess = private,GetAccess = private, Dependent)
      heading; %actual heading as calculated from x and y
    end

    methods
        % Constructor
        function uav = UAVSim()
            uav.x = 0;
            uav.y = 0.01; %Can't start at (0,0) as then dividing by zero
        end
        
        % Calculate the heading from xy position
        function heading = get.heading(uav)
            heading = mod(atan2d(uav.x, uav.y)+360, 360); 
        end
       
        % Take a sensor reading with the gps to find x and y positions
        function g = gps_sensor(uav)
            g = zeros(3,1);
            g(1) = uav.x + (rand()-0.5)*6; %plus/minus 3m gps noise
            g(2) = uav.y + (rand()-0.5)*6; %plus/minus 3m gps noise
            g(3) = mod(atan2d(g(1), g(2))+360, 360); % heading calculated from gps sensor data so as to include noise
        end
            
        function p = cloud_sensor(uav, cloud, t)
            p = cloudsamp(cloud,uav.x,uav.y,t);
        end     
            
        function move(uav, u, dt)
            %runge kutta 4th order
            state = [uav.x; uav.y; uav.heading];
            k1 = f_continuous(state,u);
            k2 = f_continuous(state+k1*dt/2,u);
            k3 = f_continuous(state+k2*dt/2,u);
            k4 = f_continuous(state+k3*dt,u);
            stateNew = state+(k1+2*k2+2*k3+k4)*dt/6;
            uav.x = stateNew(1);
            uav.y = stateNew(2);
        end
        
        function plotUAV(uav)
            plot(uav.x, uav.y, 'o')
        end
    end
    
end

