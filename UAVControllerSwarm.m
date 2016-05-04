classdef UAVControllerSwarm < handle
    %UAV controller class
    properties
        posMemory;
        posCurrent;
        target;
        state;
        launched;
        u;
        id;
        insideCloud;
        obstacle;
    end
    
    properties (Constant)
        desiredCloud = 1;
    end
    
    methods
        
        % Constructor
        function controller = UAVControllerSwarm()
            controller.posMemory = [0;0;0];
            controller.state = 1;
            controller.launched = 0;
            controller.insideCloud = 0;
        end
        
        function txtMsg = nav_decision(controller, p, kk, dt, rxMsgs)
            
            switch controller.state
                case 1 %state 1, head to target
                    if controller.outside_bounds()==1
                        controller.state=1;
                    end
                    display('Moving towards target')
                    controller.u = controller.move_to_target();
                    if norm(controller.posCurrent(1:2) - controller.target)<40;
                        if abs(controller.desiredCloud-p)<0.15
                            controller.state =3; %inside cloud
                        else
                            controller.state = 2; %start exploring
                        end
                    end
                    if ~isempty(rxMsgs)
                        [distance, neighbour] = controller.find_closest_neighbour(rxMsgs);
                        if distance<100
                            controller.obstacle = neighbour;
                            controller.state = 5;
                        end
                        controller.neighbour_inside_cloud(rxMsgs);
                        controller.insideCloud = 0;
                    end
                    
                case 2 %state 2, spiral explore
                    if controller.outside_bounds()==1
                        controller.state=1;
                    end
                    display('exploring for cloud')
                    controller.u = [20;
                        (1/10)*exp(-dt*(kk-1)*0.01)*180/pi];
                    if abs(controller.desiredCloud-p)<0.15
                        controller.state=3;
                    end
                    if ~isempty(rxMsgs)
                        [distance, neighbour] = controller.find_closest_neighbour(rxMsgs);
                        if distance<100
                            controller.obstacle = neighbour;
                            controller.state = 5;
                        end
                        controller.neighbour_inside_cloud(rxMsgs);
                        controller.insideCloud = 0;
                    end
                    
                case 3 %state 3, inside cloud boundary
                    if controller.outside_bounds()==1
                        controller.state=1;
                    end
                    display('inside cloud')
                    controller.u = [10; 6]; %spin!
                    if abs(controller.desiredCloud-p)>0.3
                        controller.rand_near_target(10);
                        controller.state=4;
                    end
                    if ~isempty(rxMsgs)
                        [distance, neighbour] = controller.find_closest_neighbour(rxMsgs);
                        if distance<100
                            controller.obstacle = neighbour;
                            controller.state = 5;
                        end
                        for mm=1:length(rxMsgs)
                            if mm==controller.id
                                continue
                            else
                                if rxMsgs(4,mm)==0
                                    controller.insideCloud = 1;
                                end
                            end
                        end
                    end
                    
                case 4 %state 4, explore current space for cloud boundary
                    if controller.outside_bounds()==1
                        controller.state=1;
                    end
                    display('Moving towards target')
                    controller.u = controller.move_to_target();
                    if abs(controller.desiredCloud-p)<0.15
                        controller.state=3; %inside the cloud boundary
                    end
                    if ~isempty(rxMsgs)
                        [distance, neighbour] = controller.find_closest_neighbour(rxMsgs);
                        if distance<100
                            controller.obstacle = neighbour;
                            controller.state = 5;
                        end
                        controller.insideCloud = 0;
                    end
                    
                case 5 %state 5, avoidance
                    if controller.outside_bounds()==1
                        controller.state=1;
                    end
                    controller.u = controller.evade();
                    if ~isempty(rxMsgs)
                        [distance, neighbour] = controller.find_closest_neighbour(rxMsgs);
                        if distance<100
                            controller.obstacle = neighbour;
                            controller.state = 5;
                        else
                            controller.state = 1;                            
                        end
                    end
            end
        controller.posMemory = controller.posCurrent;
        txtMsg = [controller.posCurrent; controller.insideCloud];
        end
        
        %Takes the gps and calculates the heading wrt previous position
        function set_current_pos(controller, gps)
            currentHeading = atan2d(gps(1)-controller.posMemory(1),...
                gps(2)-controller.posMemory(2));
            controller.posCurrent = [gps(1); gps(2); currentHeading];
        end
        
        function outside = outside_bounds(controller)
            if (abs(controller.posCurrent(1))>900) || (abs(controller.posCurrent(2))>900)
                   outside = 1;
            else
                outside = 0;
            end
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
        
        function u = evade(controller)
            headingToObstacle = controller.keep_in_range(atan2d(controller.obstacle(1)-controller.posMemory(1),...
                controller.obstacle(2)-controller.posMemory(2)) - controller.posCurrent(3) +90);
            v = 20-10*(abs(headingToObstacle)/180);
            mu = (headingToObstacle)/v/3;
            u=[v;mu];
        end
        
        
        function rand_target(controller)
        x = randi([-800,800]);
        y = randi([-800, 800]);
        controller.target = [x;y];
        end        
        
        % returns a random target within range of the current position
        function rand_near_target(controller, range)
            x = controller.posCurrent(1)+randi([-range,range]);
            y = controller.posCurrent(2)+randi([-range, range]);
            controller.target = [x;y];
        end
        
        function [distance, neighbour] = find_closest_neighbour(controller, rxMsgs)
            distances = zeros(1,length(rxMsgs));
            for mm=1:length(rxMsgs)
                if mm==controller.id
                    continue
                end
                distances(mm)=norm(controller.posCurrent(1:2)-rxMsgs(1:2, mm));
            end
            [distance, neighbourI] = max(distances);
            neighbour = rxMsgs(1:3, neighbourI);
        end
        
        function neighbour_inside_cloud(controller, rxMsgs)
            for  mm=1:length(rxMsgs)
                if mm==controller.id
                    continue
                end
                if rxMsgs(4, mm)==1
                    controller.target = rxMsgs(1:2, mm);
                    controller.state =1;
                end
            end
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




