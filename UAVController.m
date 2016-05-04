classdef UAVController < handle
    %UAV controller class
    properties
        posMemory;
        posCurrent;
        target;
        state;
    end
    
    properties (Constant)
        desiredCloud = 1;
    end
    
    methods
        
        % Constructor
        function controller = UAVController()
            controller.posMemory = [0;0;0];
            controller.state = 1;
        end
        
        function u = nav_decision(controller, p, kk, dt)
            txMsg = [];
            switch controller.state                                 
                case 1 %state 1, spiral explore
                    display('exploring for cloud')
                    u = [20;
                        (1/100)*exp(-dt*(kk-1)*0.01)*180/pi];
                    if abs(controller.desiredCloud-p)<0.15
                        controller.state=2;
                    end
                case 2 %state 2, inside cloud boundary
                    display('inside cloud')
                    u = [10; 6]; %spin!
                    if abs(controller.desiredCloud-p)>0.3
                        controller.rand_target(10);
                        controller.state=3;
                    end
                    
                case 3 %state 3, explore current space for cloud boundary
                    display('Moving towards target')
                    u = controller.move_to_target();
                    if abs(controller.desiredCloud-p)<0.15
                        controller.state=2; %inside the cloud boundary
                    end                 
            end
            controller.posMemory = controller.posCurrent;
        end
        
        %Takes the gps and calculates the heading wrt previous position
        function set_current_pos(controller, gps)
            currentHeading = atan2d(gps(1)-controller.posMemory(1),...
                gps(2)-controller.posMemory(2));
            controller.posCurrent = [gps(1); gps(2); currentHeading];
        end
        
        % Moves towards a given target
        function u = move_to_target(controller)
            headingToTarget = controller.keep_in_range(atan2d(controller.target(1)-controller.posMemory(1),...
                controller.target(2)-controller.posMemory(2)) - controller.posCurrent(3));
            v = 20-10*(abs(headingToTarget)/180);
            mu = headingToTarget/v/3;
%             v = 10*((pi/2 - abs(headingToTarget))/(pi/2));
%             mu = (3*pi/180)*(headingToTarget/(pi/2));
            u = [v;mu];
        end               
                     
        % returns a random target within range of the current position
        function rand_target(controller, range)
            x = controller.posCurrent(1)+randi([-range,range]);
            y = controller.posCurrent(2)+randi([-range, range]);
            controller.target = [x;y];
        end  
        
        function target_from_memory(controller)
            x = controller.posMemory(1) + randi([-5,5]);
            y = controller.posMemory(2) + randi([-5,5]);
            controller.target = [x; y];
        end
                             
    end
    
    methods (Static)
        function angle = keep_in_range(angle)
            %input angle in degrees and want to keep it in range [-180,180]
            if angle >180
                angle = angle-360;
            elseif angle<-180
                angle = angle+360;
            end
        end
    end
    
end



