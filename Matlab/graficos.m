close all; clc; clear

%% Convergencia
fob_iteration_GWO = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/convergencia_GWO.txt');
fob_iteration_BAT = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/convergencia_BAT.txt');
fob_iteration_AOA = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/convergencia_AOA.txt');
fob_iteration_SSA = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/convergencia_SSA.txt');
fob_iteration_PSO = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/convergencia_PSO.txt');

%% Melhores fitness
bestFit_GWO = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/best_fit_GWO.txt');
bestFit_BAT = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/best_fit_BAT.txt');
bestFit_AOA = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/best_fit_AOA.txt');
bestFit_SSA = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/best_fit_SSA.txt');
bestFit_PSO = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/best_fit_PSO.txt');
t = size(fob_iteration_GWO,2);

[Best_score_GWO,I_GWO] = min(bestFit_GWO);
[Best_score_BAT,I_BAT] = min(bestFit_BAT);
[Best_score_AOA,I_AOA] = min(bestFit_AOA);
[Best_score_SSA,I_SSA] = min(bestFit_SSA);
[Best_score_PSO,I_PSO] = min(bestFit_PSO);

%% Tempos simulação
tempo_GWO = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/Tempo_GWO.txt');
tempo_BAT = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/Tempo_BAT.txt');
tempo_AOA = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/Tempo_AOA.txt');
tempo_SSA = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/Tempo_SSA.txt');
tempo_PSO = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/Tempo_PSO.txt');
%%%%%%%%% MÉDIA DAS CURVAS
for i=1:1000
 fob_mean_GWO(i)= mean(fob_iteration_GWO(:,i));   
fob_mean_Bat(i)= mean(fob_iteration_BAT(:,i));
fob_mean_AOA(i)= mean(fob_iteration_AOA(:,i));
fob_mean_SSA(i)= mean(fob_iteration_SSA(:,i));
fob_mean_PSO(i)= mean(fob_iteration_PSO(:,i));
end
[Best_min_mean_score_GWO,I_min_mean_GWO] = min(fob_mean_GWO);
[Best_min_mean_BAT,I_min_mean_BAT] = min(fob_mean_Bat);
[Best_min_mean_AOA,I_min_mean_AOA] = min(fob_mean_AOA);
[Best_min_mean_SSA,I_min_mean_SSA] = min(fob_mean_SSA);
[Best_min_mean_PSO,I_min_mean_PSO] = min(fob_mean_PSO);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - CONVERGENCIA - MÉDIA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Position',[500 500 660 290])
%Draw objective space
subplot(1,3,1);
plot(1,fob_mean_SSA(1,1),'r--o')
set(gca,'FontSize',20)
title('(a) LM')
xlabel('Iteration','FontSize',20);
ylabel('Best score obtained so far','FontSize',20);
axis ([-20 length(fob_mean_GWO)+100 min([Best_min_mean_score_GWO Best_min_mean_BAT Best_min_mean_AOA Best_min_mean_SSA Best_min_mean_PSO]) fob_mean_GWO(1,1)+100000])
grid on
box on

subplot(1,3,2);
plot(1:t,fob_mean_SSA,'Color','r')
set(gca,'FontSize',20)
title(['(b) SSA '])
xlabel('Iteration','FontSize',20);
ylabel('Best score obtained so far','FontSize',20);
axis ([-20 length(fob_mean_SSA)+100 min([Best_min_mean_score_GWO Best_min_mean_BAT Best_min_mean_AOA Best_min_mean_SSA Best_min_mean_PSO]) fob_mean_SSA(1)+100000])
grid on
box on

% 

subplot(1,3,3);
plot(1:t,fob_mean_Bat,'Color','r')
set(gca,'FontSize',20)
title(['(c) BA'])
xlabel('Iteration','FontSize',20);
ylabel('Best score obtained so far','FontSize',20);
axis ([-20 length(fob_mean_Bat)+100 min([Best_min_mean_score_GWO Best_min_mean_BAT Best_min_mean_AOA Best_min_mean_SSA Best_min_mean_PSO]) fob_mean_Bat(1)+100000])
grid on
box on




figure('Position',[500 500 660 290])

subplot(1,3,1);

plot(1:t,fob_mean_GWO,'Color','r')
set(gca,'FontSize',20)
title(['(d) GWO'])
xlabel('Iteration','FontSize',20);
ylabel('Best score obtained so far','FontSize',20);
axis ([-20 length(fob_mean_GWO)+100 min([Best_min_mean_score_GWO Best_min_mean_BAT Best_min_mean_AOA Best_min_mean_SSA Best_min_mean_PSO]) fob_mean_GWO(1,1)+100000])
grid on
box on

subplot(1,3,2);

plot(1:t,fob_mean_AOA,'Color','r')
set(gca,'FontSize',20)
title(['(e) AOA '])
xlabel('Iteration','FontSize',20);
ylabel('Best score obtained so far','FontSize',20);
axis ([-20 length(fob_mean_AOA)+100 min([Best_min_mean_score_GWO Best_min_mean_BAT Best_min_mean_AOA Best_min_mean_SSA Best_min_mean_PSO]) fob_mean_AOA(1)+100000])
grid on
box on

subplot(1,3,3);

plot(1:t,fob_mean_PSO,'Color','r')
set(gca,'FontSize',20)
title([' (f) PSO '])
xlabel('Iteration','FontSize',20);
ylabel('Best score obtained so far','FontSize',20);
axis ([-20 length(fob_mean_PSO)+100 min([Best_min_mean_score_GWO Best_min_mean_BAT Best_min_mean_AOA Best_min_mean_SSA Best_min_mean_PSO]) fob_mean_PSO(1)+100000])
grid on
box on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONVERGENCIA%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure,
%Draw objective space

subplot(1,3,1);
plot(1,fob_iteration_BAT(1,1),'r--o')
set(gca,'FontSize',20)
title('(a) LM')
xlabel('Iteration','FontSize',20);
ylabel('Best score obtained so far','FontSize',20);
axis ([-20 length(fob_mean_GWO)+100 min([Best_min_mean_score_GWO Best_min_mean_BAT Best_min_mean_AOA Best_min_mean_SSA Best_min_mean_PSO]) fob_iteration_BAT(1)+100000])
grid on
box on

subplot(1,3,2);

plot(1:t,fob_iteration_SSA)
set(gca,'FontSize',20)
title(['(b) SSA '])
xlabel('Iteration','FontSize',20);
ylabel('Best score obtained so far','FontSize',20);
axis ([-20 length(fob_iteration_SSA)+100 min([Best_score_GWO Best_score_BAT Best_score_AOA Best_score_SSA Best_score_PSO]) fob_iteration_SSA(1)+100000])
grid on
box on

% 
subplot(1,3,3);
plot(1:t,fob_iteration_BAT)
set(gca,'FontSize',20)
title(['(c) BA'])
xlabel('Iteration','FontSize',20);
ylabel('Best score obtained so far','FontSize',20);
axis ([-20 length(fob_iteration_BAT)+100 min([Best_score_GWO Best_score_BAT Best_score_AOA Best_score_SSA Best_score_PSO]) fob_iteration_BAT(1)+100000])
grid on
box on



figure,
subplot(1,3,1);
plot(1:t,fob_iteration_GWO)
set(gca,'FontSize',20)
title(['(d) GWO'])
xlabel('Iteration','FontSize',20);
ylabel('Best score obtained so far','FontSize',20);
axis ([-20 length(fob_iteration_GWO)+100 min([Best_score_GWO Best_score_BAT Best_score_AOA Best_score_SSA Best_score_PSO]) fob_iteration_GWO(1)+100000])
grid on
box on

subplot(1,3,2);

plot(1:t,fob_iteration_AOA)
set(gca,'FontSize',20)
title(['(e) AOA '])
xlabel('Iteration','FontSize',20);
ylabel('Best score obtained so far','FontSize',20);
axis ([-20 length(fob_iteration_AOA)+100 min([Best_score_GWO Best_score_BAT Best_score_AOA Best_score_SSA Best_score_PSO]) fob_iteration_AOA(1)+100000])
grid on
box on

subplot(1,3,3);

plot(1:t,fob_iteration_PSO)
set(gca,'FontSize',20)
title([' (f) PSO '])
xlabel('Iteration','FontSize',20);
ylabel('Best score obtained so far','FontSize',20);
axis ([-20 length(fob_iteration_PSO)+100 min([Best_score_GWO Best_score_BAT Best_score_AOA Best_score_SSA Best_score_PSO]) fob_iteration_PSO(1)+100000])
grid on
box on


%% box plot

figure,
 boxplot([bestFit_GWO bestFit_BAT bestFit_AOA bestFit_SSA bestFit_PSO],  'Labels',{'GWO','BA','AOA','SSA','PSO'},'symbol', '')
set(gca,'FontSize',16)
ylabel('Fitness Function')
%title('Miles per Gallon for All Vehicles')

%% Tempo medio por simulação

mean_time_GWO = mean(tempo_GWO)
mean_time_BAT = mean(tempo_BAT)
mean_time_AOA = mean(tempo_AOA)
mean_time_SSA = mean(tempo_SSA)
mean_time_PSO = mean(tempo_PSO)



%% Dados estasticos da fitness
% Desvio padrão
std_GWO = std(bestFit_GWO)
std_Bat = std(bestFit_BAT)
std_AOA = std(bestFit_AOA)
std_SSA = std(bestFit_SSA)
std_PSO = std(bestFit_PSO)

% Maximo e Minimo
max_GWO = max(bestFit_GWO)
max_Bat = max(bestFit_BAT)
max_AOA = max(bestFit_AOA)
max_SSA = max(bestFit_SSA)
max_PSO = max(bestFit_PSO)

% Média

mean_GWO = mean(bestFit_GWO)
mean_Bat = mean(bestFit_BAT)
mean_AOA = mean(bestFit_AOA)
mean_SSA = mean(bestFit_SSA)
mean_PSO = mean(bestFit_PSO)

median_GWO = median(bestFit_GWO)
median_Bat = median(bestFit_BAT)
median_AOA = median(bestFit_AOA)
median_SSA = median(bestFit_SSA)
median_PSO = median(bestFit_PSO)
