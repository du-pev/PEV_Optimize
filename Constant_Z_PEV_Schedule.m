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

%%

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

%{
spy(A_depart)
pbaspect([1 1 1])
xlabel('N*K*2')
ylabel('N*K')
d = round(nnz(A_depart)/numel(A_depart)*100,2);
title(strcat('A Submatrix - Departure Time (d=',num2str(d),'%)'))
saveas(gcf,'A_depart_spy.png')
%}

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

%{
spy(A_req)
pbaspect([1 1 1])
xlabel('N*K*2')
ylabel('N')
d = round(nnz(A_req)/numel(A_req)*100,2);
title(strcat('A Submatrix - Requested Charge (d=',num2str(d),'%)'))
saveas(gcf,'A_req_spy.png')
%}

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


%{
spy(A_supply)
pbaspect([1 1 1])
xlabel('N*K*2')
ylabel('K')
d = round(nnz(A_supply)/numel(A_supply)*100,2);
title(strcat('A Submatrix - Peak Load (d=',num2str(d),'%)'))
saveas(gcf,'A_supply_spy.png')
%}


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

%{
spy(A_min)
pbaspect([1 1 1])
xlabel('N*K*2')
ylabel('N*K')
d = round(nnz(A_min)/numel(A_min)*100,2);
title(strcat('A Submatrix - Min/Max SOC (d=',num2str(d),'%)'))
saveas(gcf,'A_minmax_spy.png')
%}

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

vtype = [];
for j = 1:(N*kTotal*2)
    vtype = [vtype 'B'];
end

%%%%%%%%% Sense for GUROBI solver %%%%%%%%
sensor = '<';
f = zeros(N*kTotal*2,1);
%f = [1; f]';

A_matrix = sparse([A_depart; -A_supply; -A_req; -A_min; A_max]);
A_matrix(:,1) = []; 

%{
spy(A_matrix)
pbaspect([1 1 2])
xlabel('N*K*2')
ylabel('K')
d = round(nnz(A_matrix)/numel(A_matrix)*100,2);
title(strcat('Complete A Matrix (d=',num2str(d),'%)'))
%}

%% Warm Start
tic
z = N*-.0748+12786.3;

model.A = A_matrix;
model.obj = f';
model.sense = sensor;
model.vtype = vtype;
model.modelsense = 'min';

params.outputflag = 0;

b_supply = load - z;
model.rhs = [b_depart; -b_supply; -b_req; -b_min; b_max];
result = gurobi(model, params);

disp(z)
disp(result);

% feasible
if result.objbound == 0
    UB = z;
    flag = 1;
% infeasible
else     
    LB = z;
    flag = 2;
end
disp(flag)
toc
%% Get UB and LB
gap = .5;
while flag == 1
    z = z - gap;
    b_supply = load - z;
    model.rhs = [b_depart; -b_supply; -b_req; -b_min; b_max];
    result = gurobi(model, params);

    disp(z)
    disp(result);
    
    if result.objbound == 0
        UB = z;
        flag = 1;
    % infeasible
    else     
        LB = z;
        flag = 0;
    end
    disp(flag)
end


while flag == 2
    z = z + gap;
    b_supply = load - z;
    model.rhs = [b_depart; -b_supply; -b_req; -b_min; b_max];
    result = gurobi(model, params);

    disp(z)
    disp(result);
    
    if result.objbound == 0
        UB = z;
        flag = 0;
    % infeasible
    else     
        LB = z;
        flag = 2;
    end
    disp(flag)
    
end

LB
UB

%% Bisection
alpha = .5;
z = (UB+LB)/2;

z_vals = [];
runtimes = [];

for iter = 1:10
    b_supply = load - z;
    model.rhs = [b_depart; -b_supply; -b_req; -b_min; b_max];
    result = gurobi(model, params);

    disp(z)
    disp(result);
    
    z_vals(iter) = z;
    runtimes(iter) = result.runtime;
    
    if result.objbound == 0
        UB = z;
        z_feas = z;
        gap = UB-LB;
        z = UB - alpha*gap;
        result_feas = result;
        disp(strcat("Iter: ",num2str(iter), " Status: Feasible"));
    % infeasible
    else     
        LB = z;
        gap = UB-LB;
        z = LB + alpha*gap;
        disp(strcat("Iter: ",num2str(iter), " Status: Infeasible"));
    end
end

format long
disp(strcat("Best feasible objective value: ",num2str(z_feas)))

z_vals'
runtimes'

%% Massage Results

schedule = result_feas.x;

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


SOC_sched = zeros(N,kTotal+1);

SOC_sched(:,1) = SOCinit;

for k = 1:kTotal
    for n = 1:N
        SOC_sched(n,k+1) = SOC_sched(n,k) + eta(n)*g(n)*charge_sched(n,k) - eta(n)*h(n)*discharge_sched(n,k);            
    end
end


%% Plot Stuff
close all

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
    p1Title = strcat(['Optimal Charge Schedule Load for ', num2str(N), ' Vehicles (Peak Load = ' num2str(z_feas), ')']);
    title(p1Title)
    hold off
    
    fig1 = strcat('CZ_Load_', num2str(N),'.png');
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
    fig2 = strcat('CZ_Sched_', num2str(N),'.png');
    saveas(gcf,fig2)
end

%%

%spy(A_supply)
%pbaspect([1 1 1])
%}
