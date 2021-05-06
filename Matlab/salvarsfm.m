close all; clear all; clc

%salvar sfm

   best_bats = load('C:/Users/julia/Pictures/geradorartesspace/scan3/bests_sol_GWO.txt');
%   best_bats = load('C:\Users\julia\Desktop\dissertação\Resultados\Res\8\Melhores_bats.txt');
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
n =9;
for n=1:size(best_bats)
 nome = ['C:/dataset3/teste/cameras_gwo',num2str(n),'.sfm'];
%     nome = ['C:\Users\julia\Desktop\dissertação\Resultados\Res\8\cameras_bat',num2str(n),'.sfm'];
   
   fileID = fopen(nome,'w');
  
%             nbytes1 = fprintf(fil
    nbytes = fprintf(fileID,'%2d \n ', 56);
   
    variable =1;
    for m=1:length(best_bats)/6
%         nbytes1 = fprintf(fileID1,' \\hline \n \textbf{%5d} ', m);
         nbytes = fprintf(fileID,'\n%5s ', nomes_imagens{m});
        teste = variable + 5;
        for b = variable:teste
            nbytes = fprintf(fileID,'%5d ',best_bats(n,b));
%            nbytes = fprintf( fileID,'&%d ',best_bats(n,b));
            variable = variable+1;
             
            
        end
        
    end
end