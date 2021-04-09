close all; clc; clear
%% Convergencia
fob_iteration_GWO = load('C:\Users\julia\Pictures\geradorartesspace\scan3\convergencia_GWO.txt');
fob_iteration_BAT = load('C:\Users\julia\Pictures\geradorartesspace\scan3\convergencia_BAT.txt');
fob_iteration_AOA = load('C:\Users\julia\Pictures\geradorartesspace\scan3\convergencia_AOA.txt');
fob_iteration_SSA = load('C:\Users\julia\Pictures\geradorartesspace\scan3\convergencia_SSA.txt');
%% Melhores fitness
bestFit_GWO = load('C:\Users\julia\Pictures\geradorartesspace\scan3\best_fit_GWO.txt');
bestFit_BAT = load('C:\Users\julia\Pictures\geradorartesspace\scan3\best_fit_BAT.txt');
bestFit_AOA = load('C:\Users\julia\Pictures\geradorartesspace\scan3\best_fit_AOA.txt');
bestFit_SSA = load('C:\Users\julia\Pictures\geradorartesspace\scan3\best_fit_SSA.txt');
[Best_score_GWO,I_GWO] = min(bestFit_GWO);
[Best_score_BAT,I_BAT] = min(bestFit_BAT);
[Best_score_AOA,I_AOA] = min(bestFit_AOA);
[Best_score_SSA,I_SSA] = min(bestFit_SSA);
t = size(fob_iteration_AOA,2);
figure('Position',[500 500 660 290])
%Draw objective space
subplot(1,4,2);
semilogy(1:t,fob_iteration_GWO)
title(['GWO: ', num2str(Best_score_GWO)])
xlabel('Iteration');
ylabel('Best score obtained so far');
axis ([-20 length(fob_iteration_GWO)+100 Best_score_GWO-10 fob_iteration_GWO(1)])
grid on
box on

% subplot(1,4,1);
% semilogy(1:t,fob_iteration_BAT)
% title(['BAT: ', num2str(Best_score_BAT)])
% xlabel('Iteration');
% ylabel('Best score obtained so far');
% axis ([-20 length(fob_iteration_BAT) min([Best_score_GWO Best_score_BAT Best_score_AOA Best_score_SSA]) fob_iteration_BAT(1)+100])
% grid on
% box on

subplot(1,4,3);
semilogy(1:t,fob_iteration_AOA)
title(['AOA: ', num2str(Best_score_AOA)])
xlabel('Iteration');
ylabel('Best score obtained so far');
axis ([-20 length(fob_iteration_AOA)+100 min([Best_score_GWO Best_score_BAT Best_score_AOA Best_score_SSA]) fob_iteration_AOA(1)+100])
grid on
box on

subplot(1,4,4);

semilogy(1:t,fob_iteration_SSA)
title(['SSA: ', num2str(Best_score_SSA)])
xlabel('Iteration');
ylabel('Best score obtained so far');
axis ([-20 length(fob_iteration_SSA)+100 min([Best_score_GWO Best_score_BAT Best_score_AOA Best_score_SSA]) fob_iteration_SSA(1)+100])
grid on
box on

%% box plot

% figure,
% 
% boxplot([bestFit_GWO bestFit_BAT bestFit_AOA bestFit_SSA],  'Labels',{'GWO','BAT','AOA','SSA'})

% % porcentagem de erro
% 
% original = fob_iteration_bat(1,1)
% 
% porcentagem_bat_aumen = original/fmin_bat;
% porcentagem_lob_aumen = original/fmin_lobo;
% 
% porcentagem_bat_diminuiu = fmin_bat/original
% porcentagem_lob_diminuiu = fmin_lobo/original
% c = categorical({'original','BAT','GWO'});
% figure,
% bar(c,[original/original porcentagem_bat_diminuiu porcentagem_lob_diminuiu])
% title('Comparação das melhores soluções em relação ao original')


