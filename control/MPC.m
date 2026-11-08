function [value_P_s,P_grid,next_SOC]=MPC(P_RES_receiding,P_D_receiding,SOC_partenza,T,zita1,zita2)
% Receding-horizon scheduling of battery power and grid exchange.
yalmip('clear')

% Grid purchase and sale prices.
C=0.3;
B=0.2;
CAP=141;

a1=0.52;
c1=0.5765;
d1=0.59896;
P_s_star=30;

a2=32.9829;
c2=0.5127;
d2=16.0545;

% Battery efficiencies, charge limits and sampling parameters.
eta_c=0.89;
eta_d=0.89;
x_smin=0.2;
x_smax=0.9;
epsilon_t=0.01;
e_f=0.15;
delta=0.25;
alpha=0.999;

M_t=50;     
m_t=-50;

% Continuous decisions and auxiliary binary variables.
P_s=sdpvar(T,1);
P_grid_in=sdpvar(T,1);
P_grid_out=sdpvar(T,1);

x=sdpvar(T,1);

z_s=sdpvar(T,1);
lambda=binvar(T,1);

%##################VINCOLI########################

% Power balance at each prediction step.
v_8=[];  
for t=1:T
    v_8=[v_8, P_RES_receiding(t)+P_s(t)-P_D_receiding(t)+(P_grid_in(t)-P_grid_out(t))==0];
end

% Battery state-of-charge prediction.
v_9 = [x(1)==SOC_partenza];
for t=1:T-1
    v_9=[v_9, x(t+1)==x(t)*alpha+((eta_c-1/eta_d)*z_s(t)*delta-(1/eta_d)*P_s(t)*delta)/CAP];
end

% Mixed-integer constraints for the charge/discharge model.
v_10=[];
for t=1:T
    v_10=[v_10, -m_t*lambda(t)<=-P_s(t)-m_t];
end

v_11=[];
for t=1:T
    v_11=[v_11, -(M_t+epsilon_t)*lambda(t)<=P_s(t)-epsilon_t];
end

v_12=[];
for t=1:T
    v_12=[v_12, z_s(t)<=M_t*lambda(t)];
end

v_13=[];
for t=1:T
    v_13=[v_13, z_s(t) >= m_t*lambda(t)];
end

v_14=[];
for t=1:T
    v_14=[v_14, z_s(t)<=-P_s(t)-m_t*(1-lambda(t))];
end

v_15=[];
for t=1:T
    v_15=[v_15, z_s(t)>=-P_s(t)-M_t*(1-lambda(t))];
end

% Battery power limits.
v_16=[];
for t=1:T
    v_16=[v_16, P_s(t)<=P_s_star];
end

    %################################
v_16bis=[];
for t=1:T
    v_16bis=[v_16bis, P_s(t)>=-P_s_star*alpha];
end

    v_16ter=[];
    for t=1:T
        v_16ter=[v_16ter, P_s(t)>=100*(c1*x(t)-d1)];
    end

V_16bis=[v_16bis v_16ter];
%################################
% Battery state-of-charge limits.
v_17=[];
for t=1:T
    v_17=[v_17, x(t)>=x_smin, x(t)<=x_smax];
end

 % Grid import and export powers are nonnegative.
 v_P_grid=[];
 for t=1:T
     v_P_grid=[v_P_grid, 0<=P_grid_in(t)];
     v_P_grid=[v_P_grid, 0<=P_grid_out(t)];
 end

% Collect the active constraints.
vincoli=[v_8, v_9, v_10, v_11, v_12, v_13, v_14, v_15, v_16, v_16bis, v_17, v_P_grid];

% Grid cost with forecast-dependent state-of-charge weighting.
objective=(sum(C*P_grid_in-B*P_grid_out-x.*(zita1*P_RES_receiding'-zita2*P_D_receiding')));

% Solve and extract the schedule and next predicted state of charge.
optimize(vincoli,objective);

value_obiettivo=value(objective);

P_grid=(value(P_grid_in)-value(P_grid_out));

value_P_s=value(P_s);
next_SOC=value(x(2));
end
