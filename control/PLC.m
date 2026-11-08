function [P_s_PLC,P_grid_PLC]=PLC(P_s, P_grid,P_RES,P_D,P_RES_noise, P_D_noise, SOC)
% Correct scheduled powers using forecast errors and battery charge.

% Inner SOC operating range for local corrections.
SOC_max_marked=0.85;
SOC_min_marked=0.25;
treshold=0;

% Compensate the actual net-demand deviation from the forecast.
if abs((P_D_noise-P_D)-(P_RES_noise-P_RES))>=treshold

    % Use the battery while its charge is inside the working range.
    if(SOC>SOC_min_marked)&&(SOC<SOC_max_marked)
        P_s_PLC=P_s+((P_D_noise-P_D)-(P_RES_noise-P_RES));
        P_grid_PLC=P_grid;
    end
    
    % Preserve a depleted battery when additional grid power is needed.
    if(SOC<=SOC_min_marked)
        if ((P_D_noise-P_D)-(P_RES_noise-P_RES))>0
            P_grid_PLC=P_grid+((P_D_noise-P_D)-(P_RES_noise-P_RES));
            P_s_PLC=P_s;
        else 
            if ((P_D_noise-P_D)-(P_RES_noise-P_RES))<=0
                P_s_PLC=P_s+((P_D_noise-P_D)-(P_RES_noise-P_RES));
                P_grid_PLC=P_grid;
            end
        end
    end
    % Send surplus power to the grid when the battery is full.
    if(SOC>=SOC_max_marked)
        if ((P_D_noise-P_D)-(P_RES_noise-P_RES))>0
            P_s_PLC=P_s+((P_D_noise-P_D)-(P_RES_noise-P_RES));
            P_grid_PLC=P_grid;
        else 
            if ((P_D_noise-P_D)-(P_RES_noise-P_RES))<=0
                P_grid_PLC=P_grid+((P_D_noise-P_D)-(P_RES_noise-P_RES));
                P_s_PLC=P_s;
            end
        end
    end

else 
    if abs(((P_D_noise-P_D)-(P_RES_noise-P_RES))/(P_D-P_RES))<treshold
    P_s_PLC=P_s;
    P_grid_PLC=P_grid;

    end
end
