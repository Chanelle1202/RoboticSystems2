classdef UAVController < handle
    %UAV controller class
    properties
        gpsMemory;
        cloudMemory;
        uMemory;
        target;
    end
    
    properties (Constant)
        desiredCloud = 1;
    end
    
    methods
        
        % Constructor
        function controller = UAVController()
            controller.gpsMemory = [0; 0.01; 0];
            controller.cloudMemory = 0;
            controller.uMemory = [0;0];
        end
        
        function u = NavDecision(controller, gps, p, kk, dt, t)
            if p<0.5 %nowhere near the cloud
                u=[10;
                    6-(1/30)*abs(cos(0.05*(kk-1)))*180/pi];
            elseif (0.5<=p) && (p<1) %Outside the cloud
                if p>= controller.cloudMemory
                    u=[15; 0];
                else
                    controller.target = controller.gpsMemory;
                    u = controller.how_to_move(gps);
                end
            else %Inside the cloud
                if p>=controller.cloudMemory
                    
            end
            controller.uMemory  = u;
        end
        
        function u = how_to_move(controller)
            difference = controller.target-gps;
            u = [10*((pi/-abs(difference(3)))/pi/2); 
                3*pi/180*(difference(3)/pi/2)];
        end           
            
            
                      
    end
    
end

