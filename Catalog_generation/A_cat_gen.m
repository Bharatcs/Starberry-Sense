clc
clear all
close all

%Enter magnitude limit
mag_limit=3


k=dlmread('hipparcos catalog/asu_hipparcos_catalog_ra_dec_deg_wo_header.txt');


star_id=[]
ra=[]
dec=[]
stars=0;
for i=1:1:118214
    n=k(i,2);
    %remove stars with magnitude less than mag_limit
    if(n <= mag_limit)

            stars = stars + 1;
            dec =[dec , k(i,4)];
            ra =[ra , k(i,3)];
            star_id=[star_id,stars];


    end

end
data=[star_id' ra' dec'];

%store the list as mag.txt
dlmwrite('mag.txt',data,'precision',20);
%c=fopen('star_table.bin','w')
%fwrite(c,data,'float');
%fclose(c)
