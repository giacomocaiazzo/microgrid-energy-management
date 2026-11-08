function SOC=battery_model(P_s_applicata, SOC_partenza)
% Discrete battery state-of-charge update for the applied power.

% Efficiencies, time step, charge retention and storage capacity.
eta_c=0.89;
eta_d=0.89;
delta=0.25;
alpha=0.9990;
CAP=141;

% Negative power charges the battery; positive power discharges it.
if P_s_applicata<0
    eta_t=eta_c;
else 
    eta_t=1/eta_d;
end

SOC=alpha*SOC_partenza-(eta_t*P_s_applicata*delta)/CAP;

end
