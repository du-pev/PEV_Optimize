%% PEV Charge Scheduling Simulation Scenario
%{

%}
clear all
close all
clc
%% Load Grid Data
load_raw = xlsread('rt_fiveminsysload_20181110.csv');

%% Model Parameters
startInd = 96;                              % start index
endInd = 166;                               % end index
LM = 1;                                     % load multiplier (for testing)
kTotal = endInd-startInd;                   % time window
loadInit = load_raw(startInd-1,3);
load = load_raw(startInd+1:endInd,3)./LM;   % background load in MW
time = load_raw(startInd+1:endInd,1);       % time (used for plots)

N = 1000;                                   % number of vehicles
a = .1*rand(N,1) + .9;                      % battery efficiency (90-100%)
b = ((60-24)*rand(N,1) + 24)*.1;                 % battery size (24-60 kWh)
eta = a./b;                                 % efficiency/batt size

SOCinit = .5*rand(N,1) + .25;               % initial SOC (25-75%)
SOCmin = .15*rand(N,1);                     % minimum SOC (0-15%)
SOCreq = round(.5*rand(N,1) + .25,2);       % required SOC (25-75%)
SOCmax = ones(N,1);                         % maximum SOC (100%)

K = randi([kTotal-18,kTotal],N,1);          % departure time (4:30-6:00PM)

g = (.5*rand(N,1)+.5)*.1;                        % charge rate
h = g;                                      % discharge rate

saver = strcat('PEV_scenario_', num2str(N), '.mat');

save(saver) 