clc
clear all
close all


%read the star list
data=dlmread('mag.txt');
id=1:size(data(:,2));
ra=data(:,2);
dec= data(:,3);
%calculate the unit vectors
x_unit=cosd(ra).*cosd(dec);
y_unit=sind(ra).*cosd(dec);
z_unit= sind(dec);


data_final=[x_unit,y_unit,z_unit];



%write the results
dlmwrite('unit_vector/unit_vector_table.txt',data_final,'precision',16)

%generate the c array 
d=fopen('unit_vector/unit_vector_table_for_cpp.txt','w');
fprintf(d,'CATALOG_SIZE = %d \n',size(data(:,2),1))
for i=1:size(data(:,2),1)
fprintf(d,'{ %f ,%f ,%f},',x_unit(i),y_unit(i),z_unit(i));
end

fclose(d);
%c=fopen('unit_vector_table.bin','w')
%fwrite(c,data_final,'float');
%fclose(c)
