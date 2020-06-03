function [ out ] = taninverse( Del_X, Del_Y )

% Purpose to give a ouput angle from -pi to +pi based on the inputs
if(Del_X >= 0)
    
    if (Del_Y >=0)
        if (Del_Y ==0)
            out = 0;
        else
            out = atan(Del_Y / Del_X);
        end
    else
        out = -1 * atan((-1 * Del_Y)/ Del_X);
    end
    
else
    
    if (Del_Y >=0)
        out = pi - (atan(Del_Y / (-1 *Del_X)));
    else
        out = -1 * (pi - (atan((-1 * Del_Y)/ (-1 *Del_X))));
    end
    
end

end

