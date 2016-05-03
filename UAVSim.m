classdef UAVSim < handle
    %UAVSim Class  
    properties (SetAccess = private,GetAccess = private)
        x; %actual position in x
        y; %actual position in y
        heading;
        history;
    end

    methods
        % Constructor
        function uav = UAVSim()
            uav.x = 0;
            uav.y = 0;
            uav.heading = 0;
            uav.history = [0,0];
        end
       
        % Take a sensor reading with the gps to find x and y positions
        function g = gps_sensor(uav)
            g = zeros(3,1);
            g(1) = uav.x + (rand()-0.5)*6; %plus/minus 3m gps noise
            g(2) = uav.y + (rand()-0.5)*6; %plus/minus 3m gps noise
        end
            
        function p = cloud_sensor(uav, cloud, t)
            p = cloudsamp(cloud,uav.x,uav.y,t);
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
            stateNew = uav.runge_kutta4(u, dt);
            uav.x = stateNew(1);
            uav.y = stateNew(2);
            uav.heading = mod(stateNew(3), 360);
        end
        
        function stateNew = runge_kutta4(uav, u, dt)
            state = [uav.x; uav.y; uav.heading];
            %runge kutta 4th order
            k1 = f_continuous(state,u);
            k2 = f_continuous(state+k1*dt/2,u);
            k3 = f_continuous(state+k2*dt/2,u);
            k4 = f_continuous(state+k3*dt,u);
            stateNew = state+(k1+2*k2+2*k3+k4)*dt/6;
        end
        
        function plotUAV(uav)
            plot(uav.x, uav.y, 'o')
            plot(uav.history(:,1), uav.history(:,2), '-')
            uav.history = [uav.history; uav.x, uav.y];
        end
    end
    
end

