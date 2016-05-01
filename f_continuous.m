function [ x_dot ] = f_continuous( state, u)
%Continous dynamics equation
%   state is [x_position; y_position] and u is [velocity; turn curvature]
x_dot = [u(1)*sind(state(3)); 
    u(1)*cosd(state(3));
    u(1)*u(2)];
end

