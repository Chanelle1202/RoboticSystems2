classdef UAVController < handle
    %UAV controller class
    properties
        posMemory;
        posCurrent;
        cloudMemory;
        cloudSpotted;
        target;
        state
        steps;
    end
    
    properties (Constant)
        desiredCloud = 1;
    end
    
    methods
        
        % Constructor
        function controller = UAVController()
            controller.posMemory = [0;0;0];
            controller.cloudMemory = 0;
            controller.state = 1;
            controller.cloudSpotted = [0;0;0;0];
            controller.steps = 0;
        end
        
        function u = nav_decision(controller, p, kk)
            switch controller.state
                case 1 %state 1, spiral explore
                    display('exploring for cloud')
                    u = [20;
                        (1/100)*exp(-(kk-1)*0.01)*180/pi];
                    if abs(controller.desiredCloud-p)<0.15
                        controller.state=2;
                    end
                case 2 %state 2, inside cloud
                    display('inside cloud')
                    u = [10; 6];
            end
            controller.posMemory = controller.posCurrent;
        end
        
        function set_current_pos(controller, gps)
            currentHeading = mod(atan2d(gps(1)-controller.posMemory(1),...
                gps(2)-controller.posMemory(1))+360, 360);
            controller.posCurrent = [gps(1); gps(2); currentHeading];
        end
        
        function u = move_to_target(controller)
            headingToTarget = mod(atan2d(controller.target(1)-controller.posMemory(1),...
                controller.target(2)-controller.posMemory(2))+360, 360) - controller.posCurrent(3);            
            u = [10*((pi/2)-abs(headingToTarget)/pi/2);
                3*pi/180*(headingToTarget/(pi/2))];
        end
            
        function rand_target(controller, range)
            x = controller.cloudSpotted(1)+randi([-range,range]);
            y = controller.cloudSpotted(2)+randi([-range, range]);
            controller.target = [x;y];
        end  
        
        function target_from_memory(controller)
            x = controller.posMemory + randi([-5,5]);
            y = controller.posMemory + randi([-5,5]);
            controller.target = [x; y];
        end
            
            
                      
    end
    
end

