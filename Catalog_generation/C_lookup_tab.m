clc

clear all
close all
%Specify the minimum and maximum separation
min_separation_arc_sec=3600
max_separation_degree=50


min_separation =min_separation_arc_sec*pi/(3600*180);

max_separation = (max_separation_degree*pi)/180;



data=dlmread('unit_vector/unit_vector_table.txt');



x=size(data(:,3))
angle_1=[];
s_1=[];
s_2=[];
 o=0;

for i=1:1:size(data(:,3))

    for j=i+1:1:size(data(:,3))



        angle= data(i,1)*data(j,1)+ data(i,2)*data(j,2) + data(i,3)*data(j,3);
        angle=acos(angle);
        angle=abs(angle);
         if((angle > min_separation)&&(angle < max_separation))

          s_1=[s_1,i];
          s_2=[s_2,j];
          angle_1=[angle_1 ,angle];

         end
      end

     end

  data_final=[s_1',s_2',angle_1'];

  data_x= sortrows(data_final,3)
  dlmwrite('angle_table/angle_table_mag.txt',data_x,'precision',16)


%c=fopen('table_angle.bin','w')
%fwrite(c,data_final,'float');
%fclose(c)
