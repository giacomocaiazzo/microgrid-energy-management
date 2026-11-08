%% Daily microgrid energy management simulation
clear
close all
clc

% Simulation horizon, tariffs and initial battery charge.
campioni_in_giorno=96;
H=[1:1:campioni_in_giorno];
T=10;

C_t=0.3;
B_t=0.2;

SOC_0=0.5;

P_s_MPC=[];
P_grid_MPC=[];
P_s_eff=[];
P_grid_eff=[];

% Forecast errors shared by both controller configurations.
disturbo_D=(rand(1,campioni_in_giorno)-0.5)*5; 
disturbo_RES=(rand(1,campioni_in_giorno))*0.5+0.6;
start_time=0;

%###############################################################

%% Solar production forecast and disturbed measurements
mu = 12*4;
sigma = 7.5;

alba = 5*4;
tramonto = 20*4;

P_RES = (1/(sigma * sqrt(2 * pi))) * exp(-0.5 * ((H - mu) / sigma).^2);

P_RES(H < alba | H > tramonto) = 0;
P_RES=P_RES*1000;
P_RES_extended=[P_RES P_RES];
P_RES_noise=P_RES.*disturbo_RES; 
P_RES_noise(H < alba | H > tramonto) = 0;
P_RES_noise=abs(P_RES_noise);

%###############################################################

%% Demand forecast and disturbed measurements
mu_morning = 8*4;
sigma_morning = 4;

mu_evening = 19*4;
sigma_evening = 5;

P_morning = (1/(sigma_morning * sqrt(2 * pi))) * exp(-0.5 * ((H - mu_morning) / sigma_morning).^2);

P_evening = (1/(sigma_evening * sqrt(2 * pi))) * exp(-0.5 * ((H - mu_evening) / sigma_evening).^2);

P_D = P_morning + P_evening;
P_D=P_D*300+10;
P_D_doppio=[P_D P_D];

P_D_noise=P_D+disturbo_D;
P_D_noise=abs(P_D_noise);

P_D_extedned=[P_D P_D];

%###############################################################

% Plot forecast and actual input profiles.
figure(1)
nexttile
title("exogenous signals")
hold on
plot([1:length(P_D)]/4 , P_D, "LineStyle", "-.","Color", "b")
plot([1:length(P_D_noise)]/4,P_D_noise,"b")
legend("forecast","actual")
ylabel("P_D[KW]")
axis([0 24 -40 60])
grid

nexttile
plot([1:length(P_RES)]/4,P_RES, "LineStyle", "-.","Color", "r")
hold on
plot([1:length(P_RES_noise)]/4,P_RES_noise,"r","LineWidth",1)
legend("forecast","actual")
xlabel("time[h]")
ylabel("P_{RES}[KW]")
axis([0 24 -40 60])
grid

%###############################################################

SOC=SOC_0;
%% MPC simulation with forecast-dependent SOC weighting
zita1=0.3;
zita2=0.02;

costi_vet=zeros(length(zita1),1);

for j=[1:length(zita1)]
    
    costo_totale=0;
    SOC=SOC_0;
    for i=H
        P_RES_receding=P_RES_extended(i:i+T-1)
        P_D_receding=P_D_extedned(i:i+T-1)
        
        % Solve the moving-horizon schedule using the current SOC.
        [P_s_MPC, P_grid_MPC, SOC_simulato_MPC]=MPC(P_RES_receding,P_D_receding,SOC,T,zita1(j),zita2(j));
        
        % Correct the first scheduled control using measured disturbances.
        [P_s_PLC,P_grid_PLC]=PLC(P_s_MPC(1), P_grid_MPC(1),P_RES(i),P_D(i),P_RES_noise(i), P_D_noise(i), SOC);
        
        if P_grid_PLC>=0
            costo_totale=costo_totale+C_t*P_grid_PLC
        else
            costo_totale=costo_totale+B_t*P_grid_PLC
        end

        % Apply the corrected power to the battery model.
        SOC=battery_model(P_s_PLC,SOC)

        % Store applied controls and actual/predicted battery charge.
        P_s_PLC_vet(i)=P_s_PLC;
        P_grid_PLC_vet(i)=P_grid_PLC;
        
        SOC_vet(i)=SOC;
        SOC_simulato_vet(i)=SOC_simulato_MPC;

        figure(2)
        title("with soft constraints")
        hold on
        plot([1:length(SOC_vet)]/4,SOC_vet,"Color",[0 0.4470 0.7410])
        plot([1:length(SOC_simulato_vet)]/4, SOC_simulato_vet,"Color",[0.8500 0.3250 0.0980])
        legend("actual","predicted")
        xlabel("time[h]")
        ylabel("State Of Charge")
        axis([0 24 0 1])
        grid
        
        figure(3)
        title("with soft constraints")
        hold on
        plot([1:length(P_grid_PLC_vet)]/4,P_grid_PLC_vet,"Color",[0.4940 0.1840 0.5560])
        plot([1:length(P_s_PLC_vet)]/4,P_s_PLC_vet,"Color",[0.4660 0.6740 0.1880])
        legend("P_{grid}", "P_s")
        xlabel("time[h]")
        ylabel("Power Exchange [KW]")
        axis([0 24 -40 60])
        grid

    end
    costi_vet(j)=costo_totale;   
end

P_s_MPC=[];
P_grid_MPC=[];
P_s_eff=[];
P_grid_eff=[];

%% Baseline MPC simulation
zita1=0;
zita2=0;
costi_vet2=zeros(length(zita1),1);

for j=[1:length(zita1)]

    costo_totale=0;
    SOC=SOC_0;
    for i=H
        P_RES_receding=P_RES_extended(i:i+T-1)
        P_D_receding=P_D_extedned(i:i+T-1)

        % Solve the moving-horizon schedule using the current SOC.
        [P_s_MPC, P_grid_MPC, SOC_simulato_MPC]=MPC(P_RES_receding,P_D_receding,SOC,T,zita1(j),zita2(j));

        % Correct the first scheduled control using measured disturbances.
        [P_s_PLC,P_grid_PLC]=PLC(P_s_MPC(1), P_grid_MPC(1),P_RES(i),P_D(i),P_RES_noise(i), P_D_noise(i), SOC);

        if P_grid_PLC>=0
            costo_totale=costo_totale+C_t*P_grid_PLC
        else
            costo_totale=costo_totale+B_t*P_grid_PLC
        end

        % Apply the corrected power to the battery model.
        SOC=battery_model(P_s_PLC,SOC)

        % Store the results for the baseline configuration.
        P_s_davvero2(i)=P_s_PLC;
        P_grid_davvero2(i)=P_grid_PLC;

        SOC_eff2(i)=SOC;
        SOC_simulato_eff2(i)=SOC_simulato_MPC;

        figure(6)
        title("without soft constraints")
        hold on
        plot([1:length(SOC_eff2)]/4,SOC_eff2,"Color",[0 0.4470 0.7410])
        plot([1:length(SOC_simulato_eff2)]/4,SOC_simulato_eff2,"Color",[0.8500 0.3250 0.0980])
        legend("actual","predicted")
        xlabel("time[h]")
        ylabel("State Of Charge")
        axis([0 24 0 1])
        grid

        figure(7)
        title("without soft constraints")
        hold on
        plot([1:length(P_grid_davvero2)]/4,P_grid_davvero2,"Color",[0.4940 0.1840 0.5560])
        plot([1:length(P_s_davvero2)]/4,P_s_davvero2,"Color",[0.4660 0.6740 0.1880])
        legend("P_{grid}", "P_s")
        xlabel("time[h]")
        ylabel("Power Exchange [KW]")
        axis([0 24 -40 60])
        grid

    end
    costi_vet2(j)=costo_totale;   
end

figure(2)
grid
figure(3)
grid

figure(6)
grid
figure(7)
grid
