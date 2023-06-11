#include<iostream>
#include<vector>
#include<math.h>
#include"basic_functions.hpp"
#include"unit_vector/unit_vector_table.hpp"
#define number_of_stars_in_catalog 8874
#define pixel_size_x_um 0.00112
#define pixel_size_y_um 0.00112
#define x_center (3280/4) 
#define y_center (2464/4)
// plate scale is 20.4446 arcsec/pix
#define platescale_in_arcsec_per_pixel 20.4446
#define radians_per_pixel (platescale_in_arcsec_per_pixel*3.141592)/(180*3600)



float  dot_product_cos(float x[3],float a[3])
{
return acos(x[0]*a[0] + x[1]*a[1] + x[2]*a[2]);
}


void cent_to_unitvector(float cent[2], float res[3])
{
float x,y,r;

  //Astrometry correction
   const float A_0_0 = 0;
   const float A_0_1 = 0;
   const float A_0_2 = -1.57786586907E-07;
   const float A_1_0 = 0;
   const float A_1_1 = -1.6004834117E-06;
   const float A_2_0 = 9.29557814562E-07;

   const float B_0_0 = 0;
   const float B_0_1 = 0;
   const float B_0_2 = -1.54658465593E-06 ;
   const float B_1_0 = 0;
   const float B_1_1 = 2.42338816678E-06;
   const float B_2_0 = -7.66872357918E-07;

   const float CRPIX1 = 820.5;
   const float CRPIX2 = 616.5; 

   float u,v;
   float U,V;
  
    u=cent[0] - CRPIX1 ;
    v=cent[1] - CRPIX2 ;
   
   U = u + A_0_0 + A_0_1*v + A_1_0*u + A_1_1*u*v + A_2_0*u*u + A_0_2*v*v + CRPIX1;
   V = v + B_0_0 + B_0_1*v + B_1_0*u + B_1_1*u*v + B_2_0*u*u + B_0_2*v*v + CRPIX2;

   x = (U - x_center)*(radians_per_pixel);
   y = (V - y_center)*(radians_per_pixel);

  r = sqrt(1 + x*x + y*y);

  res[0]=x/r;
  res[1]=y/r;
  res[2]=1/r;

}

float  dot_product_cos_using_id(int i,int j)
{

if((i > 0) && (j> 0) && (i< number_of_stars_in_catalog+1) && (i< number_of_stars_in_catalog+1))
return acos(unit_vector_table[i-1][0]*unit_vector_table[j-1][0] + unit_vector_table[i-1][1]*unit_vector_table[j-1][1] + unit_vector_table[i-1][2]*unit_vector_table[j-1][2] );
else return 0;
}
