close all; clear all; clc


%% Melhores fitness
bestFit_GWO = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/best_fit_GWO.txt');
bestFit_BAT = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/best_fit_BAT.txt');
bestFit_AOA = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/best_fit_AOA.txt');
bestFit_SSA = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/best_fit_SSA.txt');
bestFit_PSO = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/best_fit_PSO.txt');


[Best_score_GWO,I_GWO] = min(bestFit_GWO);
[Best_score_BAT,I_BAT] = min(bestFit_BAT);
[Best_score_AOA,I_AOA] = min(bestFit_AOA);
[Best_score_SSA,I_SSA] = min(bestFit_SSA);
[Best_score_SSA,I_PSO] = min(bestFit_PSO);
%salvar sfm

best_GWO = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/bests_sol_GWO.txt');
best_bats = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32\bests_sol_BAT.txt');
best_AOA = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32\bests_sol_AOA.txt');
best_SSA = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32\bests_sol_SSA.txt');
best_PSO = load('C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32\bests_sol_PSO.txt');

nomes_imagens = cell(1, length(best_bats)/6);

for n=1:length(best_bats)/6
    if n < 10
        nome = ['C:/dataset3/imagem_00',num2str(n),'.png'];
    else
        nome = ['C:/dataset3/imagem_0',num2str(n),'.png'];
    end
    nomes_imagens{n} = nome;
end

teste=1;
variable =1;

% for n=1:size(best_bats)
nome_GWO = ['C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/cameras_GWO',num2str(I_GWO),'.sfm'];
nome_BAT = ['C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/cameras_BAT',num2str(I_BAT),'.sfm'];
nome_AOA = ['C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/cameras_AOA',num2str(I_AOA),'.sfm'];
nome_SSA = ['C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/cameras_SSA',num2str(I_SSA),'.sfm'];
nome_PSO = ['C:\Users\julia\Pictures\SantosDumont_4\santosdumont_patio\scan32/cameras_PSO',num2str(I_PSO),'.sfm'];


file_GWO  = fopen(nome_GWO,'w');
file_BAT = fopen(nome_BAT,'w');
file_AOA = fopen(nome_AOA,'w');
file_SSA = fopen(nome_SSA,'w');
file_PSO = fopen(nome_PSO,'w');

nbytes_GWO = fprintf(file_GWO,'%2d \n ', 56);
nbytes_BAT = fprintf(file_BAT,'%2d \n ', 56);
nbytes_AOA = fprintf(file_AOA,'%2d \n ', 56);
nbytes_SSA = fprintf(file_SSA,'%2d \n ', 56);
nbytes_PSO = fprintf(file_PSO,'%2d \n ', 56);
variable =1;
for m=1:length(best_bats)/6
    
    nbytes_GWO = fprintf(file_GWO,'\n%5s ', nomes_imagens{m});
    nbytes_BAT = fprintf(file_BAT,'\n%5s ', nomes_imagens{m});
    nbytes_AOA = fprintf(file_AOA,'\n%5s ', nomes_imagens{m});
    nbytes_SSA = fprintf(file_SSA,'\n%5s ', nomes_imagens{m});
    nbytes_PSO = fprintf(file_PSO,'\n%5s ', nomes_imagens{m});
    
    
    teste = variable + 5;
    for b = variable:teste
        nbytes_GWO = fprintf(file_GWO,'%5d ',best_GWO(I_GWO,b));
        nbytes_BAT = fprintf(file_BAT,'%5d ',best_bats(I_BAT,b));
        nbytes_AOA = fprintf(file_AOA,'%5d ',best_AOA(I_AOA,b));
        nbytes_SSA = fprintf(file_SSA,'%5d ',best_SSA(I_SSA,b));
        nbytes_PSO = fprintf(file_PSO,'%5d ',best_PSO(I_PSO,b));
        variable = variable+1;
        
        
    end
    
end
% end