clc
clear all
close all

data=dlmread('angle_table/angle_table_mag.txt')



id_1=data(:,1);
id_2=data(:,2);
ang= data(:,3);


data_final=[ang,id_1,id_2];
%distance search table for cpp 
d=fopen('angle_table/search_table_angle_for_cpp.txt','w');
fprintf(d,'SEARCH_TABLE_SIZE = %d \n',size(data(:,1),1))
fprintf(d,'%f,',ang);
fclose(d);
%star ID pair search table for cpp to the corresponding distances
d=fopen('angle_table/search_table_id_for_cpp.txt','w');
fprintf(d,'SEARCH_TABLE_SIZE = %d \n',size(data(:,1),1))
for i=1:size(ang)
fprintf(d,'{%d,%d},',id_1(i),id_2(i));
end
fclose(d);
