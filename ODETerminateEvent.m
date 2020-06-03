function [value, isTerminal, direction] = ODETerminateEvent(t, Y)
    global R_lethal;
    value = Y(1)- R_lethal;
    isTerminal  = 1;   % Stop the integration
    direction   = -1;
end