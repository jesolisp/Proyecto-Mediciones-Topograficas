clc
clear all
close all

d = load ("C:\Users\rayoy\Documents\MATLAB\Libro_1.csv");
mm = d(:,1); %Guarda los datos de la primer columna
lecturas = d(:,2); %Guarda datos de la segunda columna
lecturas2 = d(:,3); %Guarda datos de la segunda columna

plot (lecturas,mm,'r');
hold on 
plot (lecturas2,mm,'b');
title('lecturas sobre le eje x')
xlabel('lecturas ida y vuelta del sensor')
ylabel('mm')
