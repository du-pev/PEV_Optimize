%% PEV Charge Scheduling Simulation Scenario
%{

%}
clear all
close all
clc
%% Load PEV Scenario
load('PEV_scenario_1000.mat')

%% Model
% Integrality Requirement:
tic

% build matrix for when each vehicle is scheduled to depart.
% after depart time, vehicle must be at least charged to their required
% charge time, and can no longer charge/discharge
depart = ones(N,kTotal);
for n = 1:N
    for k = K(n):kTotal
        depart(n,k) = 0;
    end
end

% turns depature matrix into b vector
b_depart = depart(:);
% creates required SOC b vector
b_req = SOCreq-SOCinit;
% creates grid load b vector
b_supply = load;

% creates min SOC and max SOC b vectors 
b_min_simple = SOCmin - SOCinit;
b_max_simple = SOCmax - SOCinit;
b_min = [];
b_max = [];
for j = 1:kTotal
    b_min = [b_min_simple; b_min];
    b_max = [b_max_simple; b_max];
end
toc

%% A_depart
tic
kInd = 2;

i = zeros(1,N*kTotal);
j = zeros(1,N*kTotal);
j2 = zeros(1,N*kTotal);
v = zeros(1,N*kTotal);

for k = 1:kTotal
    for n = 1:N    
        
        i(n+(k-1)*N) = n+(k-1)*N;       
        j(n+(k-1)*N) = kInd;
        j2(n+(k-1)*N) = kInd+1;
        v(n+(k-1)*N) = 1;
            
        kInd = kInd + 2;        
    end
end

A_depart1 = sparse(i,j,v,N*kTotal,2*N*kTotal+1);
A_depart2 = sparse(i,j2,v,N*kTotal,2*N*kTotal+1);
A_depart = A_depart1 + A_depart2;
toc

%% A_req
tic
kInd = 2;

i = zeros(1,N*kTotal);
j = zeros(1,N*kTotal);
j2 = zeros(1,N*kTotal);
v1 = zeros(1,N*kTotal);
v2 = zeros(1,N*kTotal);

for k = 1:kTotal
    for n = 1:N    
        
        i(n+(k-1)*N) = n;       
        j(n+(k-1)*N) = kInd;
        j2(n+(k-1)*N) = kInd+1;
        v1(n+(k-1)*N) = eta(n)*g(n);
        v2(n+(k-1)*N) = -eta(n)*h(n);
        
        kInd = kInd + 2;        
    end
end

A_req1 = sparse(i,j,v1,N,2*N*kTotal+1);
A_req2 = sparse(i,j2,v2,N,2*N*kTotal+1);
A_req = A_req1 + A_req2;
toc

%% A_supply
tic

nInd = 2;

i = zeros(1,N*kTotal);
i2 = zeros(1,kTotal);
j1 = zeros(1,N*kTotal);
j2 = zeros(1,N*kTotal);
v1 = zeros(1,N*kTotal);
v2 = zeros(1,N*kTotal);
v3 = zeros(1,kTotal);

for k = 1:kTotal
    for n = 1:N    
        
        i(n+(k-1)*N) = k;       
        j1(n+(k-1)*N) = nInd;
        j2(n+(k-1)*N) = nInd+1;
        v1(n+(k-1)*N) = -g(n);
        v2(n+(k-1)*N) = h(n);
                
        nInd = nInd + 2;
            
    end
    i2(k) = k;
    v3(k) = 1;
end

A_supply1 = sparse(i,j,v1,kTotal,2*N*kTotal+1);
A_supply2 = sparse(i,j2,v2,kTotal,2*N*kTotal+1);
A_supply3 = sparse(i2,1,v3,kTotal,2*N*kTotal+1);
A_supply = A_supply1 + A_supply2 + A_supply3;
toc

%% A_min
tic

nInd = 2;

i = zeros(1,N*kTotal*(kTotal+1)/2);
j1 = zeros(1,N*kTotal*(kTotal+1)/2);
j2 = zeros(1,N*kTotal*(kTotal+1)/2);
v1 = zeros(1,N*kTotal*(kTotal+1)/2);
v2 = zeros(1,N*kTotal*(kTotal+1)/2);
counter = 1;

for k = 1:kTotal
    for n = 1:N                            
        for c = 1:kTotal        
            if (k <= c)
                i(counter) = n+(c-1)*N;
                j1(counter) = nInd;
                j2(counter) = nInd+1;
                v1(counter) = eta(n)*g(n);
                v2(counter) = -eta(n)*h(n);  
                counter = counter+1;
            end
        end        
        nInd = nInd + 2;
    end    
end

A_min1 = sparse(i,j1,v1,kTotal*N,2*N*kTotal+1);
A_min2 = sparse(i,j2,v2,kTotal*N,2*N*kTotal+1);
A_min = A_min1 + A_min2;
A_max = A_min;
toc

%% Plots    
if (0)
    plot(time,load)
    ylim([10000,15000]);
    xlim([time(1),time(endInd-startInd)]);
    datetick('x','HHPM','keeplimits')
end
    
%% Optimize!
% Copyright 2018, Gurobi Optimization, LLC
% This example formulates and solves the following simple MIP model:
%  maximize
%        x +   y + 2 z
%  subject to
%        x + 2 y + 3 z <= 4
%        x +   y       >= 1
%  x, y, z binary

vtype = ['C'];
for j = 1:(N*kTotal*2)
    vtype = [vtype 'B'];
end

%%%%%%%%% Sense for GUROBI solver %%%%%%%%
sensor = '<';
f = zeros(N*kTotal*2,1);
f = [1; f]';

model.A = sparse([A_depart; -A_supply; -A_req; -A_min; A_max]);
model.obj = f;
model.rhs = [b_depart; -b_supply; -b_req; -b_min; b_max];
model.sense = sensor;
model.vtype = vtype;
model.modelsense = 'min';

%gurobi_write(model, 'mip1.lp');

params.outputflag = 0;

result = gurobi(model, params);

disp(result);

%% Massage Results
schedule = result.x;
schedule(1) = [];

charge_sched = [];
discharge_sched = [];
tot_sched = [];

for k = 1:kTotal
    for n = 1:N
        charge_sched(n,k) = schedule((n*2-1)+((k-1)*N*2));
        discharge_sched(n,k) = schedule((n*2)+((k-1)*N*2));
        
        tot_sched(n,k*2-1) = schedule((n*2-1)+((k-1)*N*2));
        tot_sched(n,k*2) = schedule((n*2)+((k-1)*N*2));
    end    
end

SOC_sched = SOCinit;

for k = 1:kTotal
    for n = 1:N
        SOC_sched(n,k+1) = SOC_sched(n,k) + eta(n)*g(n)*charge_sched(n,k) - eta(n)*h(n)*discharge_sched(n,k);            
    end
end

%% Plot Stuff
E = zeros(kTotal,1);
E(1) = 0;

for k = 1:kTotal    
    veh_sum = 0;
    for n = 1:N
        veh_sum = veh_sum + g(n)*charge_sched(n,k) - h(n)*discharge_sched(n,k);        
    end
    E(k) = veh_sum;
end

optiLoad = load + E;
[max_optiLoad,ind_optiLoad] = max(optiLoad);

%% Plots    
if (1)
    figure (1)
    hold on
    plot(time,load)    
    xlim([time(1),time(endInd-startInd)]);
    datetick('x','HHPM','keeplimits')
    plot(time,optiLoad,'-p','MarkerIndices',[ind_optiLoad],...
    'MarkerFaceColor','red',...
    'MarkerSize',15)
    xlabel('Time (hours)');
    ylabel('Load (MW)');
    legend('Forecasted Load','Optimal Charge Schedule Load')
    p1Title = strcat(['Optimal Charge Schedule Load for ', num2str(N), ' Vehicles (Peak Load = ' num2str(result.objval), ')']);
    title(p1Title)
    hold off
    
    fig1 = strcat('Load_', num2str(N),'.png');
    set(gcf, 'Position', [50 50 1000 800])
    saveas(gcf,fig1)
end

if (1)
    figure (2)
    hold on
    plot(time,SOC_sched(4,2:end),'r')
    plot(time,SOC_sched(5,2:end),'b')
    plot(time,SOC_sched(6,2:end),'g')
    xlim([time(1),time(endInd-startInd)]);
    datetick('x','HHPM','keeplimits')
        
    rl1 = refline(0,SOCreq(4));
    rl2 = refline(0,SOCreq(5));
    rl3 = refline(0,SOCreq(6));
    rl1.LineStyle = '--';
    rl2.LineStyle = '--';
    rl3.LineStyle = '--';
    rl1.Color = 'r';
    rl2.Color = 'b';
    rl3.Color = 'g';
    
    p2Title = strcat(['State of Charge of 3 (Out of ', num2str(N), ') Vehicles']);
    title(p2Title)
    ylabel('State of Charge (%)')
    xlabel('Time (hours)')
    legend('SOC V1','SOC V2','SOC V3','REQ V1','REQ V2','REQ V3')
    
    set(gcf, 'Position', [50 50 1000 800])
    fig2 = strcat('Sched_', num2str(N),'.png');
    saveas(gcf,fig2)
end
    

    
    