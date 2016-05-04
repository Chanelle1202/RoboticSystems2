function [ stateNew ] = runge_kutta4( state, u, dt)
%runge kutta 4th order
                k1 = f_continuous(state,u);
                k2 = f_continuous(state+k1*dt/2,u);
                k3 = f_continuous(state+k2*dt/2,u);
                k4 = f_continuous(state+k3*dt,u);
                stateNew = state+(k1+2*k2+2*k3+k4)*dt/6;

end

